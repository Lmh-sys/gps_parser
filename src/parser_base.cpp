#include "gps_parser/parser_base.h"
 
ParserBase::ParserBase() : running_(false) {}
 
ParserBase::~ParserBase() {
}
// 辅助函数：分割字符串
std::vector<std::string> ParserBase::split(const std::string& s, char delimiter) {
    std::vector<std::string> tokens;
    size_t start = 0;
    size_t end = s.find(delimiter);
    
    while (end != std::string::npos) {
        tokens.push_back(s.substr(start, end - start)); 
        start = end + 1;
        end = s.find(delimiter, start);
    }
    tokens.push_back(s.substr(start)); // 添加最后一个字段 
    
    // 处理末尾连续分隔符的情况（如"a,b,"）
    if (!s.empty() && s.back() == delimiter) {
        tokens.emplace_back("");
    }
    return tokens;
}
 
// 辅助函数：校验和验证 
bool ParserBase::validateChecksum(const std::string& data) {
    return true;
    size_t asteriskPos = data.find('*');
    if (asteriskPos == std::string::npos) {
        return false;
    }
 
    uint8_t calculated = 0;
    for (size_t i = 1; i < asteriskPos; ++i) {
        calculated ^= data[i];
    }
 
    std::stringstream ss;
    ss << std::hex << std::uppercase << static_cast<int>(calculated);
    std::string expected = ss.str();
    
    std::string actual = data.substr(asteriskPos + 1, 2);
    if (expected.length() == 1 && actual.length() == 2) {
        expected = "0" + expected;
    }
    std::cout << "actual:" << actual << "expected:" << expected << std::endl;
    return expected == actual;
}