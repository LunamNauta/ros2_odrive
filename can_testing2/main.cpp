#include <locale>
#include <unordered_map>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <regex>

struct CAN_Signal{
    std::string name;
    std::size_t bit_position;
    std::size_t bit_length;
    bool endianness;
    bool signedness;
    float scaling;
    float offset;
    float minimum;
    float maximum;
    std::string units;
    std::vector<std::string> receivers;
};

struct CAN_Object_Header{
    std::size_t index;
    std::string name;
    std::size_t length;
    std::vector<std::string> senders;
};
struct CAN_Object{
    CAN_Object_Header header;
    std::vector<CAN_Signal> signals;
};

std::string::const_iterator Read_Spaces(const std::string::const_iterator& beg, const std::string::const_iterator& end){
    return std::find_if(beg, end, [](char c){return !std::isspace(c);});
}
std::string::const_iterator Read_Unsigned(const std::string::const_iterator& beg, const std::string::const_iterator& end){
    return std::find_if(beg, end, [](char c){return !std::isdigit(c);});
}
std::string::const_iterator Read_Until(const std::string::const_iterator& beg, const std::string::const_iterator& end, char val){
    return std::find_if(beg, end, [&val](char c){return c == val;});
}

// Spec found here: https://github.com/Aceinna/docs_aceinna-dmu380/blob/master/media/Vector_DBC_File_Format_Documentation.pdf
// These functions, as written, do not follow this spec
int Read_CAN_Object_Header_Line(const std::string& line, CAN_Object_Header* output){
    std::string::const_iterator it1, it2, it3;

    it1 = Read_Spaces(line.begin(), line.end());
    if (std::string(it1, it1+4) != "BO_ ") return -1;
    it1 = Read_Spaces(it1 += 4, line.end());
    it2 = Read_Unsigned(it1, line.end());
    if (it2 == it1) return -1;
    output->index = std::stoull(std::string(it1, it2));

    it1 = Read_Spaces(it1 = it2, line.end());
    it2 = Read_Until(it1, line.end(), ':');
    if (it2 == line.end()) return -1;
    it3 = Read_Until(it1, line.end(), ' ');
    it2 = std::min(it2, it3);
    if (it2 == it1) return -1;
    output->name = std::string(it1, it2);

    it1 = Read_Spaces(it1 = it3+1, line.end());
    it2 = Read_Unsigned(it1, line.end());
    if (it2 == it1) return -1;
    output->length = std::stoull(std::string(it1, it2));

    while ((it1 = it2) != line.end()){
        it1 = Read_Spaces(it1, line.end());
        it2 = Read_Until(it1, line.end(), ' ');
        if (it2 == it1) break;
        output->senders.push_back(std::string(std::string(it1, it2)));
    }

    return 0;
}
int Read_CAN_Signal_Line(const std::string& line, CAN_Signal* output){
    std::string::const_iterator it1, it2, it3;

    it1 = Read_Spaces(line.begin(), line.end());
    if (std::string(it1, it1+4) != "SG_ ") return -1;
    it1 = Read_Spaces(it1 += 4, line.end());
    it2 = Read_Until(it1, line.end(), ' ');
    if (it2 == it1) return -1;
    output->name = std::string(it1, it2);

    it1 = Read_Spaces(it1 = it2, line.end());
    if (*it1 != ':') return -1;
    it1 = Read_Spaces(++it1, line.end());
    it2 = Read_Unsigned(it1, line.end());
    if (it2 == it1) return -1;
    output->bit_position = std::stoull(std::string(it1, it2));

    if (*(it1 = it2) != '|') return -1;
    it2 = Read_Unsigned(++it1, line.end());
    if (it2 == it1) return -1;
    output->bit_length = std::stoull(std::string(it1, it2));

    if (*((it1 = it2)++) != '@') return -1;
    if (*it1 != '0' && *it1 != '1') return -1;
    output->endianness = *it1 - '0';

    if (*(++it1) != '+' && *it1 != '-') return -1;
    output->signedness = *it1 == '-';

    it1 = Read_Spaces(++it1, line.end());
    if (*it1 != '(') return -1;
    it2 = Read_Unsigned(++it1, line.end());
    if (it2 == it1) return -1;
    output->scaling = std::stoull(std::string(it1, it2));
    if (*it2 != ',') return -1;
    it2 = Read_Unsigned(it1 = it2+1, line.end());
    if (it2 == it1) return -1;
    output->offset = std::stoull(std::string(it1, it2));
    if (*(it1 = it2) != ')') return -1;

    it1 = Read_Spaces(++it1, line.end());
    if (*it1 != '[') return -1;
    it2 = Read_Unsigned(++it1, line.end());
    if (it2 == it1) return -1;
    output->minimum = std::stoull(std::string(it1, it2));
    if (*it2 != '|') return -1;
    it2 = Read_Unsigned(it1 = it2+1, line.end());
    if (it2 == it1) return -1;
    output->maximum = std::stoull(std::string(it1, it2));
    if (*(it1 = it2) != ']') return -1;

    it1 = Read_Spaces(++it1, line.end());
    if (*it1 != '\"') return -1;
    it2 = Read_Until(++it1, line.end(), '\"');
    if (it2 != it1) output->units = std::string(it1, it2);

    it1 = Read_Spaces(it1 = it2+1, line.end());
    it2 = Read_Until(it1, line.end(), ' ');
    if (it2 == it1) return -1;
    output->receivers.push_back(std::string(it1, it2));

    while ((it1 = it2) != line.end()){
        it1 = Read_Spaces(it1, line.end());
        it2 = Read_Until(it1, line.end(), ' ');
        if (it2 == it1) break;
        output->receivers.push_back(std::string(std::string(it1, it2)));
    }

    return 0;
}

int main(){
    std::fstream input_file("odrive_cansimple.dbc");
    std::string line;

    std::vector<CAN_Object> can_objects;
    CAN_Object_Header header;
    CAN_Signal signal;

    while (std::getline(input_file, line)){
        int err = 0;

        err = Read_CAN_Object_Header_Line(line, &header);
        if (err >= 0){
            can_objects.push_back({header, {}});
            continue;
        }
        err = Read_CAN_Signal_Line(line, &signal);
        if (err >= 0 && can_objects.size()){
            can_objects.back().signals.push_back(signal);
            continue;
        }
    }

    for (const CAN_Object& can_object : can_objects){
        std::cout << can_object.header.name << ":\n";
        for (const CAN_Signal& can_signal : can_object.signals){
            std::cout << "    " << can_signal.name << "\n";
        }
        std::cout << "\n";
    }
}
