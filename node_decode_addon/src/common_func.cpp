#include "common_func.h"

vector<string> split(const string& s, const string& sep)
{
    vector<string> v; 
    string::size_type pos1, pos2;
    pos2 = s.find(sep);
    pos1 = 0;
    while(string::npos != pos2)
    {
        v.push_back(s.substr(pos1, pos2-pos1));
         
        pos1 = pos2 + sep.size();
        pos2 = s.find(sep, pos1);
    }
    if(pos1 != s.length())
        v.push_back(s.substr(pos1));
    return v;
}

void split2double(char* string,char* delim,double* value_array,size_t array_len) {
	size_t i = 0;
	char* p = strtok(string, delim);
	while (p != NULL && i < array_len) {//当返回值不为NULL时，继续循环
		value_array[i] = atof(p);
		p = strtok(NULL, delim);//继续调用strtok，分解剩下的字符串
		i++;
	}
}