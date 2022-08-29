#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#define FATAL_ERROR(msg, ...) { fprintf(stderr, msg "\n", ##__VA_ARGS__); exit(-1); }
typedef uint32_t UInt32;
typedef uint16_t UInt16;
typedef uint8_t Byte;
typedef UInt16 CProb;

#define kTopValue ((UInt32)1 << 24)
#define kNumBitModelTotalBits 11
#define PROB_INIT_VAL ((1 << kNumBitModelTotalBits) / 2)
#define INIT_PROBS(p) \
 { for (unsigned i = 0; i < sizeof(p) / sizeof(p[0]); i++) p[i] = PROB_INIT_VAL; }
#define kNumMoveBits 5


#define LZMA_DIC_MIN (1 << 12)

enum LzmaError {
    LZMA_RES_NO_ERROR,
    LZMA_RES_FINISHED_WITHOUT_MARKER,
    LZMA_RES_ERROR,
    LZMA_RES_FINISHED_WITH_MARKER
};

unsigned lc, pb, lp;
UInt32 dictSize;
UInt32 dictSizeInProperties;

void DecodeProperties(const Byte *properties) {
    unsigned d = properties[0];
    if (d >= (9 * 5 * 5))
      throw "Incorrect LZMA properties";
    lc = d % 9;
    d /= 9;
    pb = d / 5;
    lp = d % 5;
    dictSizeInProperties = 0;
    for (int i = 0; i < 4; i++)
        dictSizeInProperties |= (UInt32)properties[i + 1] << (8 * i);
    dictSize = dictSizeInProperties;
    if (dictSize < LZMA_DIC_MIN)
        dictSize = LZMA_DIC_MIN;
}


struct CRangeDecoder {
    uint32_t Range;
    uint32_t Code;
    bool Corrupted;
    FILE *InStream;

    int Init(FILE *in) {
        Corrupted = false;
        Range = 0xFFFFFFFF;
        Code = 0;
        InStream = in;

        int b = fgetc(InStream);

        for (int i = 0; i < 4; i++)
            Code = (Code << 8) | fgetc(in);

        if (b != 0 || Code == Range)
            Corrupted = true;
        return b == 0;
    }

    bool IsFinishedOK() const { return Code == 0; }

    void Normalize() {
        if (Range < kTopValue) {
            Range <<= 8;
            Code = (Code << 8) | fgetc(InStream);
        }
    }

    UInt32 DecodeDirectBits(unsigned numBits) {
        UInt32 res = 0;
        do {
            Range >>= 1;
            Code -= Range;
            UInt32 t = 0 - ((UInt32)Code >> 31);
            Code += Range & t;

            if (Code == Range)
                Corrupted = true;

            Normalize();
            res <<= 1;
            res += t + 1;
        } while (--numBits);
        return res;
    }

    unsigned DecodeBit(CProb *prob) {
        unsigned v = *prob;
        UInt32 bound = (Range >> kNumBitModelTotalBits) * v;
        unsigned symbol;
        if (Code < bound) {
            v += ((1 << kNumBitModelTotalBits) - v) >> kNumMoveBits;
            Range = bound;
            symbol = 0;
        }
        else {
            v -= v >> kNumMoveBits;
            Code -= bound;
            Range -= bound;
            symbol = 1;
        }
        *prob = (CProb)v;
        Normalize();
        return symbol;
    }
};

CRangeDecoder RangeDec;


int get_bit(FILE *in) {
    static int byte;
    static unsigned bit_mask = 0;

    if (bit_mask == 0) {
        byte = fgetc(in);
        if (byte == EOF) {
            return byte;
        }

        bit_mask = 1;
    }

    int rv = byte & bit_mask;
    bit_mask <<= 1;
    return rv;
}


// void read_header(FILE *in) {
//     int prop = fgetc(in);
//     if (prop == EOF) FATAL_ERROR("Out of data reading header properties.");
//     if (prop > (4 * 5 + 4) * 9 + 8)
//         FATAL_ERROR("Header properties invalid");
// 
//     unsigned num_pos_bits = prop / (9 * 5);
//     prop -= num_pos_bits * 9 * 5;
//     unsigned num_literal_pos_bits = prop / 9;
//     unsigned num_literal_context_bits = prop - num_literal_pos_bits * 9;
// 
//     printf("num_pos_bits: %d\n", num_pos_bits);
//     printf("num_literal_pos_bits: %d\n", num_literal_pos_bits);
//     printf("num_literal_context_bits: %d\n", num_literal_context_bits);
// 
//     uint32_t dict_size_bytes;
//     if (fread(&dict_size_bytes, 4, 1, in) != 1)
//         FATAL_ERROR("Out of data reading header dict size.");
//     printf("Dict size: %d\n", dict_size_bytes);
// 
//     uint64_t uncompressed_size_bytes;
//     if (fread(&uncompressed_size_bytes, 8, 1, in) != 1)
//         FATAL_ERROR("Out of data reading header uncompressed size.");
//     if (uncompressed_size_bytes == -1)
//         printf("Uncompressed size: unknown");
//     else
//         printf("Uncompressed size: %ld\n", uncompressed_size_bytes);
// }


class COutWindow {
    Byte *Buf;
    UInt32 Pos;
    UInt32 Size;
    bool IsFull;

public:
    unsigned TotalPos;
    FILE *OutStream;

    COutWindow(): Buf(NULL) {}
    ~COutWindow() { delete []Buf; }

    void Create(UInt32 dictSize) {
        Buf = new Byte[dictSize];
        Pos = 0;
        Size = dictSize;
        IsFull = false;
        TotalPos = 0;
    }

    void PutByte(Byte b) {
        TotalPos++;
        Buf[Pos++] = b;
        if (Pos == Size)
        {
            Pos = 0;
            IsFull = true;
        }
        fputc(b, OutStream);
    }

    Byte GetByte(UInt32 dist) const {
        return Buf[dist <= Pos ? Pos - dist : Size - dist + Pos];
    }

    void CopyMatch(UInt32 dist, unsigned len) {
        for (; len > 0; len--)
            PutByte(GetByte(dist));
    }

    bool CheckDistance(UInt32 dist) const {
        return dist <= Pos || IsFull;
    }

    bool IsEmpty() const {
        return Pos == 0 && !IsFull;
    }
};

COutWindow OutWindow;


unsigned BitTreeReverseDecode(CProb *probs, unsigned numBits, CRangeDecoder *rc) {
    unsigned m = 1;
    unsigned symbol = 0;
    for (unsigned i = 0; i < numBits; i++) {
        unsigned bit = rc->DecodeBit(&probs[m]);
        m <<= 1;
        m += bit;
        symbol |= (bit << i);
    }
    return symbol;
}

template <unsigned NumBits>
class CBitTreeDecoder {
    CProb Probs[(unsigned)1 << NumBits];

public:
    void Init() { INIT_PROBS(Probs); }

    unsigned Decode(CRangeDecoder *rc) {
        unsigned m = 1;
        for (unsigned i = 0; i < NumBits; i++)
            m = (m << 1) + rc->DecodeBit(&Probs[m]);
        return m - ((unsigned)1 << NumBits);
    }

    unsigned ReverseDecode(CRangeDecoder *rc) {
        return BitTreeReverseDecode(Probs, NumBits, rc);
    }
};


  CProb *LitProbs;

  void CreateLiterals() {
    LitProbs = new CProb[(UInt32)0x300 << (lc + lp)];
  }

  void InitLiterals() {
    UInt32 num = (UInt32)0x300 << (lc + lp);
    for (UInt32 i = 0; i < num; i++)
      LitProbs[i] = PROB_INIT_VAL;
  }

  void DecodeLiteral(unsigned state, UInt32 rep0) {
    unsigned prevByte = 0;
    if (!OutWindow.IsEmpty())
      prevByte = OutWindow.GetByte(1);

    unsigned symbol = 1;
    unsigned litState = ((OutWindow.TotalPos & ((1 << lp) - 1)) << lc) + (prevByte >> (8 - lc));
    CProb *probs = &LitProbs[(UInt32)0x300 * litState];

    if (state >= 7) {
      unsigned matchByte = OutWindow.GetByte(rep0 + 1);
      do {
        unsigned matchBit = (matchByte >> 7) & 1;
        matchByte <<= 1;
        unsigned bit = RangeDec.DecodeBit(&probs[((1 + matchBit) << 8) + symbol]);
        symbol = (symbol << 1) | bit;
        if (matchBit != bit)
          break;
      }
      while (symbol < 0x100);
    }
    while (symbol < 0x100)
      symbol = (symbol << 1) | RangeDec.DecodeBit(&probs[symbol]);
    OutWindow.PutByte((Byte)(symbol - 0x100));
  }


#define kNumPosBitsMax 4

class CLenDecoder {
  CProb Choice;
  CProb Choice2;
  CBitTreeDecoder<3> LowCoder[1 << kNumPosBitsMax];
  CBitTreeDecoder<3> MidCoder[1 << kNumPosBitsMax];
  CBitTreeDecoder<8> HighCoder;

public:
  void Init() {
    Choice = PROB_INIT_VAL;
    Choice2 = PROB_INIT_VAL;
    HighCoder.Init();
    for (unsigned i = 0; i < (1 << kNumPosBitsMax); i++) {
      LowCoder[i].Init();
      MidCoder[i].Init();
    }
  }

  unsigned Decode(CRangeDecoder *rc, unsigned posState) {
    if (rc->DecodeBit(&Choice) == 0)
      return LowCoder[posState].Decode(rc);
    if (rc->DecodeBit(&Choice2) == 0)
      return 8 + MidCoder[posState].Decode(rc);
    return 16 + HighCoder.Decode(rc);
  }
};

CLenDecoder LenDecoder;
CLenDecoder RepLenDecoder;


// match distance decoding

#define kNumLenToPosStates 4
#define kEndPosModelIndex 14
#define kNumFullDistances (1 << (kEndPosModelIndex >> 1))
#define kNumAlignBits 4

CBitTreeDecoder<6> PosSlotDecoder[kNumLenToPosStates];
CProb PosDecoders[1 + kNumFullDistances - kEndPosModelIndex];
CBitTreeDecoder<kNumAlignBits> AlignDecoder;

void InitDist() {
    for (unsigned i = 0; i < kNumLenToPosStates; i++)
        PosSlotDecoder[i].Init();
    AlignDecoder.Init();
    INIT_PROBS(PosDecoders);
}

unsigned DecodeDistance(unsigned len) {
    unsigned lenState = len;
    if (lenState > kNumLenToPosStates - 1)
        lenState = kNumLenToPosStates - 1;

    unsigned posSlot = PosSlotDecoder[lenState].Decode(&RangeDec);
    if (posSlot < 4)
        return posSlot;

    unsigned numDirectBits = (unsigned)((posSlot >> 1) - 1);
    UInt32 dist = ((2 | (posSlot & 1)) << numDirectBits);
    if (posSlot < kEndPosModelIndex)
        dist += BitTreeReverseDecode(PosDecoders + dist - posSlot, numDirectBits, &RangeDec);
    else {
        dist += RangeDec.DecodeDirectBits(numDirectBits - kNumAlignBits) << kNumAlignBits;
        dist += AlignDecoder.ReverseDecode(&RangeDec);
    }
    return dist;
}

#define kNumStates 12
#define kNumPosBitsMax 4

unsigned UpdateState_Literal(unsigned state) {
  if (state < 4) return 0;
  else if (state < 10) return state - 3;
  else return state - 6;
}

unsigned UpdateState_Match   (unsigned state) { return state < 7 ? 7 : 10; }
unsigned UpdateState_Rep     (unsigned state) { return state < 7 ? 8 : 11; }
unsigned UpdateState_ShortRep(unsigned state) { return state < 7 ? 9 : 11; }

#define kMatchMinLen 2


LzmaError MainLoop(bool unpackSizeDefined, unsigned unpackSize, bool markerIsMandatory) {
    // Last 4 match distances.
    UInt32 rep0 = 0, rep1 = 0, rep2 = 0, rep3 = 0;

    CProb IsMatch[kNumStates << kNumPosBitsMax];
    CProb IsRep[kNumStates];
    CProb IsRepG0[kNumStates];
    CProb IsRepG1[kNumStates];
    CProb IsRepG2[kNumStates];
    CProb IsRep0Long[kNumStates << kNumPosBitsMax];

    unsigned state = 0;

    while (1) {
        // Check "end of stream" conditions.
        if (unpackSizeDefined && unpackSize == 0 && !markerIsMandatory)
            if (RangeDec.IsFinishedOK())
                return LZMA_RES_FINISHED_WITHOUT_MARKER;

        unsigned posState = OutWindow.TotalPos & ((1 << pb) - 1);
        unsigned state2 = (state << kNumPosBitsMax) + posState;

        if (IsMatch[state2] == 0) {
            // Literal found.
            if (unpackSizeDefined && unpackSize == 0)
                return LZMA_RES_ERROR;

            DecodeLiteral(state, rep0);

            state = UpdateState_Literal(state);
            unpackSize--;
        } else {
            if (IsRep[state] == 0) {
                // Simple Match
                rep3 = rep2;
                rep2 = rep1;
                rep1 = rep0;
                unsigned len = LenDecoder.Decode(&RangeDec, posState);
                state = UpdateState_Match(state);
                rep0 = DecodeDistance(len);
                if (rep0 == 0xFFFFFFFF)
                    return RangeDec.IsFinishedOK() ?
                        LZMA_RES_FINISHED_WITH_MARKER :
                        LZMA_RES_ERROR;
                if (unpackSizeDefined && unpackSize == 0)
                    return LZMA_RES_ERROR;
                if (rep0 >= dictSize || !OutWindow.CheckDistance(rep0))
                    return LZMA_RES_ERROR;

                // Copy matched symbols
                len += kMatchMinLen;
                bool isError = false;
                if (unpackSizeDefined && unpackSize < len) {
                    len = (unsigned)unpackSize;
                    isError = true;
                }
                OutWindow.CopyMatch(rep0 + 1, len);
                unpackSize -= len;
                if (isError)
                    return LZMA_RES_ERROR;
            }
            else {
                printf("Unsupported\n");
                return LZMA_RES_NO_ERROR;
//               1 - Rep Match
//                 IsRepG0[state] decode
//                   0 - the distance is rep0
//                     IsRep0Long[state2] decode
//                       0 - Short Rep Match
//                       1 - Rep Match 0
//                   1 -
//                     IsRepG1[state] decode
//                       0 - Rep Match 1
//                       1 -
//                         IsRepG2[state] decode
//                           0 - Rep Match 2
//                           1 - Rep Match 3
            }
        }

//       Decode Type of MATCH / LITERAL.
//         If it's a LITERAL, decode LITERAL value and write the LITERAL to the
//             sliding window.
//         If it's MATCH, decode the length of match and the match distance.
//             Check error conditions, check end of stream conditions and copy
//             the sequence of match bytes from the sliding window to the current
//             position in the sliding window.
    }
}

int main() {
    FILE *in = fopen("hello.txt.lzma", "rb");

    Byte header[13];
    fread(header, 13, 1, in);
    DecodeProperties(header);
    printf("lc=%u pb=%u lp=%u dictSize=%u dictSizeInProperties=%u\n",
        lc, pb, lp, dictSize, dictSizeInProperties);

    uint64_t unpackSize = *((uint64_t*)(header + 5));
    bool unpackSizeDefined = true;
    if (unpackSize == -1l) {
        printf("unpacked size not defined\n");
        unpackSizeDefined = false;
    }
    else {
        printf("unpacked size defined = %u\n", (unsigned)unpackSize);
    }

    MainLoop(unpackSizeDefined, unpackSize, true);
}
