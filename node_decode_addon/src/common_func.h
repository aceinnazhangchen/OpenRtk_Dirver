#ifndef COMMON_H
#define COMMON_H

#include <string>
#include <vector>

using namespace std;

vector<string> split(const string& s, const string& sep);

void split2double(char* string,char* delim,double* value_array,size_t array_len);

#endif
