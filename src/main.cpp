/*
 * PNG to ASCII art converter
 *
 * Build:
 *   g++ -O2 -std=c++17 -o png2ascii png2ascii.cpp 
 *   ||
 *   xmake build
 *
 * Usage:
 *   png2ascii [options] image.png
 *   Options:
 *     -w  <int>    Output width  in chars (default: 80)
 *     -H  <int>    Output height in chars (0 = auto from aspect ratio)
 *     -s  <name>   Charset style: standard | dense | simple | binary (default: standard)
 *     -i           Invert brightness
 *     -c           ANSI 24-bit colour output
 *     --help
 *
 * Examples:
 *   png2ascii photo.png
 *   png2ascii -w 120 -s dense photo.png
 *   png2ascii -w 60 -H 30 -s simple -i logo.png
 *   png2ascii -w 100 -c photo.png
 */

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iostream>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

using u8  = uint8_t;
using u16 = uint16_t;
using u32 = uint32_t;

// ─────────────────────────── helpers ────────────────────────────────────────

static u32 be32(const u8* p) {
    return (u32(p[0]) << 24) | (u32(p[1]) << 16) | (u32(p[2]) << 8) | p[3];
}

// ─────────────────────────── bit reader (LSB-first) ─────────────────────────

struct BitReader {
    const std::vector<u8>& data;
    size_t pos;
    int bit;

    BitReader(const std::vector<u8>& d, size_t start = 0)
        : data(d), pos(start), bit(0) {}

    int read_bit() {
        if (pos >= data.size()) throw std::runtime_error("deflate: unexpected EOF");
        int b = (data[pos] >> bit) & 1;
        if (++bit == 8) { bit = 0; ++pos; }
        return b;
    }

    u32 read_bits(int n) {
        u32 v = 0;
        for (int i = 0; i < n; ++i) v |= read_bit() << i;
        return v;
    }

    void align() { if (bit) { bit = 0; ++pos; } }

    u16 read_u16_le() {
        u8 lo = data[pos++], hi = data[pos++];
        return u16(lo) | u16(u16(hi) << 8);
    }
};

// ─────────────────────────── Huffman decoder ────────────────────────────────

struct Huffman {
    int maxlen = 0;
    std::map<std::pair<u32, int>, int> tbl;

    void build(const std::vector<int>& lens) {
        maxlen = 0;
        for (int l : lens) if (l > maxlen) maxlen = l;
        if (!maxlen) return;

        std::vector<int> cnt(maxlen + 1, 0);
        for (int l : lens) if (l) cnt[l]++;

        std::vector<u32> nxt(maxlen + 1, 0);
        u32 code = 0;
        for (int b = 1; b <= maxlen; ++b) {
            code = (code + cnt[b - 1]) << 1;
            nxt[b] = code;
        }

        tbl.clear();
        for (int sym = 0; sym < (int)lens.size(); ++sym) {
            int l = lens[sym];
            if (l) { tbl[{nxt[l], l}] = sym; nxt[l]++; }
        }
    }

    int decode(BitReader& br) const {
        u32 code = 0;
        for (int b = 1; b <= maxlen; ++b) {
            code = (code << 1) | br.read_bit();
            auto it = tbl.find({code, b});
            if (it != tbl.end()) return it->second;
        }
        throw std::runtime_error("deflate: bad Huffman code");
    }
};

// ─────────────────────────── deflate tables ─────────────────────────────────

static const int LBASE[29]  = {3,4,5,6,7,8,9,10,11,13,15,17,19,23,27,31,
                                35,43,51,59,67,83,99,115,131,163,195,227,258};
static const int LEXTRA[29] = {0,0,0,0,0,0,0,0,1,1,1,1,2,2,2,2,
                                3,3,3,3,4,4,4,4,5,5,5,5,0};
static const int DBASE[30]  = {1,2,3,4,5,7,9,13,17,25,33,49,65,97,129,193,
                                257,385,513,769,1025,1537,2049,3073,4097,6145,
                                8193,12289,16385,24577};
static const int DEXTRA[30] = {0,0,0,0,1,1,2,2,3,3,4,4,5,5,6,6,
                                7,7,8,8,9,9,10,10,11,11,12,12,13,13};

static Huffman make_fixed_lit() {
    std::vector<int> lens(288);
    for (int i = 0;   i <= 143; ++i) lens[i] = 8;
    for (int i = 144; i <= 255; ++i) lens[i] = 9;
    for (int i = 256; i <= 279; ++i) lens[i] = 7;
    for (int i = 280; i <= 287; ++i) lens[i] = 8;
    Huffman h; h.build(lens); return h;
}

static Huffman make_fixed_dist() {
    std::vector<int> lens(32, 5);
    Huffman h; h.build(lens); return h;
}

// ─────────────────────────── inflate ────────────────────────────────────────

static void inflate_codes(BitReader& br, const Huffman& lh,
                           const Huffman& dh, std::vector<u8>& out) {
    while (true) {
        int sym = lh.decode(br);
        if (sym < 256) {
            out.push_back(u8(sym));
        } else if (sym == 256) {
            break;
        } else {
            int li  = sym - 257;
            int len = LBASE[li] + (int)br.read_bits(LEXTRA[li]);
            int di  = dh.decode(br);
            int dst = DBASE[di] + (int)br.read_bits(DEXTRA[di]);
            size_t base = out.size() - dst;
            for (int k = 0; k < len; ++k) out.push_back(out[base + k]);
        }
    }
}

static std::vector<u8> inflate(const std::vector<u8>& zdata) {
    // skip 2-byte zlib header (CMF + FLG)
    BitReader br(zdata, 2);
    std::vector<u8> out;
    out.reserve(zdata.size() * 3);

    static Huffman flit  = make_fixed_lit();
    static Huffman fdist = make_fixed_dist();
    static const int CLEN_ORDER[19] = {16,17,18,0,8,7,9,6,10,5,11,4,12,3,13,2,14,1,15};

    bool final_block = false;
    while (!final_block) {
        final_block = br.read_bits(1);
        int btype   = (int)br.read_bits(2);

        if (btype == 0) {                               // no compression
            br.align();
            u16 len  = br.read_u16_le();
            /* nlen = */ br.read_u16_le();
            for (int i = 0; i < len; ++i) out.push_back(zdata[br.pos++]);

        } else if (btype == 1) {                        // fixed Huffman
            inflate_codes(br, flit, fdist, out);

        } else if (btype == 2) {                        // dynamic Huffman
            int hlit  = (int)br.read_bits(5) + 257;
            int hdist = (int)br.read_bits(5) + 1;
            int hclen = (int)br.read_bits(4) + 4;

            std::vector<int> clens(19, 0);
            for (int i = 0; i < hclen; ++i)
                clens[CLEN_ORDER[i]] = (int)br.read_bits(3);
            Huffman clh; clh.build(clens);

            std::vector<int> all(hlit + hdist, 0);
            for (int i = 0; i < (int)all.size(); ) {
                int sym = clh.decode(br);
                if (sym < 16) {
                    all[i++] = sym;
                } else if (sym == 16) {
                    int prev = all[i - 1];
                    int rep  = (int)br.read_bits(2) + 3;
                    for (int r = 0; r < rep; ++r) all[i++] = prev;
                } else if (sym == 17) {
                    i += (int)br.read_bits(3) + 3;
                } else {
                    i += (int)br.read_bits(7) + 11;
                }
            }

            Huffman lh, dh;
            lh.build(std::vector<int>(all.begin(), all.begin() + hlit));
            dh.build(std::vector<int>(all.begin() + hlit, all.end()));
            inflate_codes(br, lh, dh, out);

        } else {
            throw std::runtime_error("deflate: reserved BTYPE");
        }
    }
    return out;
}

// ─────────────────────────── PNG decoder ────────────────────────────────────

struct Image {
    int w = 0, h = 0;
    int bit_depth = 8, color_type = 0, channels = 1;
    // Normalised to 8-bit-per-channel after decoding.
    // Layout: rows[y] has w * channels bytes.
    std::vector<std::vector<u8>> rows;
};

static int paeth_pred(int a, int b, int c) {
    int p = a + b - c;
    int pa = std::abs(p - a), pb = std::abs(p - b), pc = std::abs(p - c);
    if (pa <= pb && pa <= pc) return a;
    return (pb <= pc) ? b : c;
}

static Image load_png(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    if (!f) throw std::runtime_error("cannot open: " + path);
    std::vector<u8> file((std::istreambuf_iterator<char>(f)), {});

    static const u8 SIG[8] = {137,80,78,71,13,10,26,10};
    if (file.size() < 8 || memcmp(file.data(), SIG, 8))
        throw std::runtime_error("not a valid PNG file");

    Image img;
    std::vector<u8> idat;
    std::vector<u8> palette;

    size_t pos = 8;
    while (pos + 12 <= file.size()) {
        u32 len  = be32(file.data() + pos); pos += 4;
        char tag[5] = {};
        memcpy(tag, file.data() + pos, 4); pos += 4;
        const u8* d = file.data() + pos;

        if (!strcmp(tag, "IHDR")) {
            img.w          = (int)be32(d);
            img.h          = (int)be32(d + 4);
            img.bit_depth  = d[8];
            img.color_type = d[9];
            if (d[10] || d[12])            // compression ≠ 0 or interlace ≠ 0
                throw std::runtime_error("unsupported PNG feature (compression/interlace)");
            switch (img.color_type) {
                case 0: img.channels = 1; break;
                case 2: img.channels = 3; break;
                case 3: img.channels = 1; break; // indexed — expanded later
                case 4: img.channels = 2; break;
                case 6: img.channels = 4; break;
                default: throw std::runtime_error("unsupported PNG color type");
            }
        } else if (!strcmp(tag, "PLTE")) {
            palette.assign(d, d + len);
        } else if (!strcmp(tag, "IDAT")) {
            idat.insert(idat.end(), d, d + len);
        } else if (!strcmp(tag, "IEND")) {
            break;
        }

        pos += len + 4; // skip CRC
    }

    if (!img.w || !img.h) throw std::runtime_error("missing IHDR");

    // bytes per sample (1 or 2 for 16-bit)
    int bps   = img.bit_depth == 16 ? 2 : 1;
    // bytes per pixel in the raw filtered stream
    int bpp   = (img.color_type == 3) ? 1 : img.channels * bps;
    int stride = img.w * bpp;

    std::vector<u8> raw = inflate(idat);
    img.rows.resize(img.h, std::vector<u8>(img.w * img.channels));

    size_t rpos = 0;
    std::vector<u8> prev(stride, 0);

    for (int y = 0; y < img.h; ++y) {
        if (rpos >= raw.size()) throw std::runtime_error("PNG: raw data too short");
        int ftype = raw[rpos++];
        std::vector<u8> row(raw.begin() + rpos, raw.begin() + rpos + stride);
        rpos += stride;

        // Reconstruct filter
        for (int x = 0; x < stride; ++x) {
            u8 a = (x >= bpp)     ? row[x - bpp]  : 0;
            u8 b =                  prev[x];
            u8 c = (x >= bpp)     ? prev[x - bpp] : 0;
            switch (ftype) {
                case 0: break;
                case 1: row[x] += a; break;
                case 2: row[x] += b; break;
                case 3: row[x] += u8((int(a) + b) / 2); break;
                case 4: row[x] += u8(paeth_pred(a, b, c)); break;
                default: throw std::runtime_error("PNG: unknown filter type");
            }
        }
        prev = row;

        // Convert to 8-bit and expand palette / 16-bit
        auto& out_row = img.rows[y];
        if (img.color_type == 3) {
            // Indexed → RGB
            out_row.resize(img.w * 3);
            for (int x = 0; x < img.w; ++x) {
                int idx = row[x] * 3;
                if (idx + 2 < (int)palette.size()) {
                    out_row[x*3]   = palette[idx];
                    out_row[x*3+1] = palette[idx+1];
                    out_row[x*3+2] = palette[idx+2];
                }
            }
        } else if (img.bit_depth == 16) {
            // 16-bit → 8-bit (take high byte)
            for (int i = 0; i < img.w * img.channels; ++i)
                out_row[i] = row[i * 2];
        } else {
            out_row = row;
        }
    }

    // Normalise color_type for indexed
    if (img.color_type == 3) { img.color_type = 2; img.channels = 3; }

    return img;
}

// ─────────────────────────── pixel -> gray ───────────────────────────────────

struct Pixel { float r, g, b; };

static Pixel get_pixel(const Image& img, int x, int y) {
    const auto& row = img.rows[y];
    float r, g, b;
    switch (img.color_type) {
        case 0: r = g = b = row[x] / 255.f; break;
        case 4: r = g = b = row[x * 2] / 255.f; break;
        case 2: r = row[x*3]/255.f; g = row[x*3+1]/255.f; b = row[x*3+2]/255.f; break;
        case 6: r = row[x*4]/255.f; g = row[x*4+1]/255.f; b = row[x*4+2]/255.f; break;
        default: r = g = b = 0; break;
    }
    return {r, g, b};
}

static float luminance(Pixel p) {
    return 0.299f * p.r + 0.587f * p.g + 0.114f * p.b;
}

// ─────────────────────────── charsets ───────────────────────────────────────

static const char* CS_STANDARD = " .,:;i1tfLCG08@#";
static const char* CS_DENSE    = " .'`^\",:;Il!i><~+_-?][}{1)(|\\/tfjrxnuvczXYUJCLQ0OZmwqpdbkhao*#MW&8%B@$";
static const char* CS_SIMPLE   = " -+#@";
static const char* CS_BINARY   = " @";

// ─────────────────────────── main ───────────────────────────────────────────
// Your changes can start from here. 
// PNG file format decoding is quite complicate in

static void usage(const char* prog) {
    std::cerr <<
        "Usage: " << prog << " [options] image.png\n"
        "\n"
        "Options:\n"
        "  -w <int>    Output width  in characters  (default: 80)\n"
        "  -H <int>    Output height in characters  (0 = auto, default: 0)\n"
        "  -s <name>   Charset style (default: standard)\n"
        "              Styles: standard | dense | simple | binary\n"
        "  -i          Invert brightness\n"
        "  -c          ANSI 24-bit colour output\n"
        "  -o <name>   Output as a txt file  (default: restul.txt)\n"
        "  --help      Show this help\n"
        "\n"
        "Examples:\n"
        "  " << prog << " photo.png\n"
        "  " << prog << " -w 120 -s dense photo.png\n"
        "  " << prog << " -w 60 -H 30 -s simple -i logo.png\n"
        "  " << prog << " -w 100 -c photo.png\n";
}

int main(int argc, char* argv[]) {
    int out_w = 80, out_h = 0;
    std::string output_name = "restul.txt";
    std::string style = "standard";
    bool invert = false, colour = false, output_txt = false;
    std::string filename;
    std::ofstream* p_file = nullptr;

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if ((a == "-w") && i + 1 < argc)       out_w  = std::stoi(argv[++i]);
        else if ((a == "-H") && i + 1 < argc)  out_h  = std::stoi(argv[++i]);
        else if ((a == "-s") && i + 1 < argc)  style  = argv[++i];
        else if (a == "-i")                     invert = true;
        else if (a == "-c")                     colour = true;
        //else if (a == "-o" && i + 1 < argc)     output_txt = true, output_name = std::string(argv[++i]);
        else if (a == "-o") {
            output_txt = true;
            if (i + 1 < argc && argv[i + 1][0] != '-') {
                output_name = std::string(argv[++i]);
            }
        }
        else if (a == "--help") { usage(argv[0]); return 0; }
        else if (a[0] != '-')                   filename = a;
        else { std::cerr << "Unknown option: " << a << "\n"; usage(argv[0]); return 1; }
    }

    if (output_txt == true && colour == true) {std::cerr << "Txt file do not allow colors" << "\n";return 0;}
    if (filename.empty()) { usage(argv[0]); return 1; }

    if (output_txt) {
        static std::ofstream output_file(output_name);
        p_file = &output_file;
        std::cout << "Output as a txt file ing...\n";
    }

    const char* cs = CS_STANDARD;
    if      (style == "dense")  cs = CS_DENSE;
    else if (style == "simple") cs = CS_SIMPLE;
    else if (style == "binary") cs = CS_BINARY;

    Image img;
    try {
        img = load_png(filename);
    } catch (const std::exception& e) {
        std::cerr << "Error loading PNG: " << e.what() << "\n";
        return 1;
    }

    // Auto height: terminal chars ~2× taller than wide
    if (out_h == 0)
        out_h = std::max(1, (int)std::round(out_w * (float)img.h / img.w * 0.45f));

    int cslen = (int)strlen(cs);

    for (int oy = 0; oy < out_h; ++oy) {
        for (int ox = 0; ox < out_w; ++ox) {
            // Map output cell → source region
            int x0 = ox       * img.w / out_w;
            int x1 = (ox + 1) * img.w / out_w;
            int y0 = oy       * img.h / out_h;
            int y1 = (oy + 1) * img.h / out_h;
            x1 = std::min(std::max(x1, x0 + 1), img.w);
            y1 = std::min(std::max(y1, y0 + 1), img.h);

            // Average over the region
            float lum = 0;
            float ar = 0, ag = 0, ab = 0;
            int cnt = 0;
            for (int y = y0; y < y1; ++y) {
                for (int x = x0; x < x1; ++x) {
                    Pixel p = get_pixel(img, x, y);
                    lum += luminance(p);
                    if (colour) { ar += p.r; ag += p.g; ab += p.b; }
                    ++cnt;
                }
            }
            lum /= cnt;
            if (invert) lum = 1.f - lum;

            int ci = (int)std::round(lum * (cslen - 1));
            ci = std::max(0, std::min(cslen - 1, ci));

            if (colour) {
                int r = (int)(ar / cnt * 255);
                int g = (int)(ag / cnt * 255);
                int b = (int)(ab / cnt * 255);
                std::cout << "\033[38;2;" << r << ";" << g << ";" << b << "m";
            }
            
            if (output_txt) {
                if (p_file) {
                (*p_file) << cs[ci];
                }
            } else {
            std::cout << cs[ci];
            }
        }
        if (colour) std::cout << "\033[0m";
        if (output_txt) {
            if (p_file) {
                (*p_file) << '\n';
                }
        } else {
        std::cout << '\n';
        }
    }
    if (output_txt){
        (*p_file).close();
        std::cout << "Output as a txt file successful";
    }

    return 0;
}