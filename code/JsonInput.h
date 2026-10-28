// Author: Feiran
// Parse JSON benchmark documents without external dependencies.

#ifndef MASP_JSON_INPUT_H
#define MASP_JSON_INPUT_H

#include <cctype>
#include <cmath>
#include <cstdlib>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

namespace masp_input {
struct Json {
    enum Kind { Null, Boolean, Number, String, Array, Object } kind = Null;
    double number = 0;
    std::string string;
    std::vector<Json> array;
    std::map<std::string, Json> object;

    const Json &required(const std::string &key) const {
        const auto found = object.find(key);
        if (kind != Object || found == object.end())
            throw std::runtime_error("missing JSON field: " + key);
        return found->second;
    }
    double numeric(const std::string &key) const {
        const Json &value = required(key);
        if (value.kind != Number)
            throw std::runtime_error("JSON field must be numeric: " + key);
        return value.number;
    }
};

// Complete JSON parsing precedes allocation of solver-specific arrays.
class JsonParser {
    const std::string &source;
    std::size_t position = 0;
    [[noreturn]] void fail() const {
        throw std::runtime_error("invalid JSON at byte " + std::to_string(position));
    }
    void space() {
        while (position < source.size() && (source[position] == ' ' || source[position] == '\n' ||
                                            source[position] == '\r' || source[position] == '\t'))
            ++position;
    }
    bool consume(char c) {
        space();
        if (position < source.size() && source[position] == c) {
            ++position;
            return true;
        }
        return false;
    }
    void scanDigits() {
        while (position < source.size() &&
               std::isdigit(static_cast<unsigned char>(source[position])))
            ++position;
    }
    unsigned hex4() {
        unsigned result = 0;
        for (int i = 0; i < 4; ++i) {
            if (position == source.size())
                fail();
            const char c = source[position++];
            result <<= 4;
            if (c >= '0' && c <= '9')
                result += c - '0';
            else if (c >= 'a' && c <= 'f')
                result += c - 'a' + 10;
            else if (c >= 'A' && c <= 'F')
                result += c - 'A' + 10;
            else
                fail();
        }
        return result;
    }
    static void utf8(std::string &out, unsigned code) {
        if (code < 0x80)
            out += char(code);
        else if (code < 0x800) {
            out += char(0xc0 | (code >> 6));
            out += char(0x80 | (code & 63));
        } else if (code < 0x10000) {
            out += char(0xe0 | (code >> 12));
            out += char(0x80 | ((code >> 6) & 63));
            out += char(0x80 | (code & 63));
        } else {
            out += char(0xf0 | (code >> 18));
            out += char(0x80 | ((code >> 12) & 63));
            out += char(0x80 | ((code >> 6) & 63));
            out += char(0x80 | (code & 63));
        }
    }
    std::string quoted() {
        if (!consume('"'))
            fail();
        std::string out;
        while (position < source.size()) {
            const unsigned char c = source[position++];
            if (c == '"')
                return out;
            if (c < 0x20)
                fail();
            if (c != '\\') {
                out += char(c);
                continue;
            }
            if (position == source.size())
                fail();
            switch (source[position++]) {
            case '"':
                out += '"';
                break;
            case '\\':
                out += '\\';
                break;
            case '/':
                out += '/';
                break;
            case 'b':
                out += '\b';
                break;
            case 'f':
                out += '\f';
                break;
            case 'n':
                out += '\n';
                break;
            case 'r':
                out += '\r';
                break;
            case 't':
                out += '\t';
                break;
            case 'u': {
                unsigned code = hex4();
                if (code >= 0xd800 && code <= 0xdbff) {
                    if (position + 2 > source.size() || source[position] != '\\' ||
                        source[position + 1] != 'u')
                        fail();
                    position += 2;
                    const unsigned low = hex4();
                    if (low < 0xdc00 || low > 0xdfff)
                        fail();
                    code = 0x10000 + (code - 0xd800) * 0x400 + low - 0xdc00;
                } else if (code >= 0xdc00 && code <= 0xdfff)
                    fail();
                utf8(out, code);
                break;
            }
            default:
                fail();
            }
        }
        fail();
    }
    Json value(unsigned depth) {
        if (depth > 128)
            fail();
        space();
        if (position == source.size())
            fail();
        Json out;
        if (consume('{')) {
            out.kind = Json::Object;
            if (consume('}'))
                return out;
            do {
                const std::string key = quoted();
                if (!consume(':'))
                    fail();
                if (!out.object.emplace(key, value(depth + 1)).second)
                    fail();
                if (consume('}'))
                    return out;
            } while (consume(','));
            fail();
        }
        if (consume('[')) {
            out.kind = Json::Array;
            if (consume(']'))
                return out;
            do {
                out.array.push_back(value(depth + 1));
                if (consume(']'))
                    return out;
            } while (consume(','));
            fail();
        }
        if (source[position] == '"') {
            out.kind = Json::String;
            out.string = quoted();
            return out;
        }
        for (const char *literal : {"true", "false", "null"}) {
            const std::string token(literal);
            if (source.compare(position, token.size(), token) == 0) {
                position += token.size();
                out.kind = token == "null" ? Json::Null : Json::Boolean;
                return out;
            }
        }
        const std::size_t start = position;
        if (source[position] == '-')
            ++position;
        if (position == source.size())
            fail();
        if (source[position] == '0')
            ++position;
        else {
            if (source[position] < '1' || source[position] > '9')
                fail();
            scanDigits();
        }
        if (position < source.size() && source[position] == '.') {
            const std::size_t digits = ++position;
            scanDigits();
            if (position == digits)
                fail();
        }
        if (position < source.size() && (source[position] == 'e' || source[position] == 'E')) {
            ++position;
            if (position < source.size() && (source[position] == '+' || source[position] == '-'))
                ++position;
            const std::size_t digits = position;
            scanDigits();
            if (position == digits)
                fail();
        }
        out.kind = Json::Number;
        out.number = std::strtod(source.substr(start, position - start).c_str(), nullptr);
        if (!std::isfinite(out.number))
            fail();
        return out;
    }

  public:
    explicit JsonParser(const std::string &input) : source(input) {}
    Json parse() {
        Json result = value(0);
        space();
        if (position != source.size())
            fail();
        return result;
    }
};
}
#endif
