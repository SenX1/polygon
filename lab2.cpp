#include <iostream>
#include <fstream>
#include <set>
#include <string>
#include <sstream>

using namespace std;
//получить массив с уникальными значениями из массива с 1e4 чисел с разделителем ; и значением от 1 до 31765
int main(){
    string nums_str; // 24b structure
    set<short int> num_arr; // 4b srtuc
    ifstream file("input.txt"); // 472b from hdd -> ram 
    getline(file, nums_str); //nums_str 24 + 2 * i
    stringstream stream(nums_str); // 24b for obj + 368b bufer (ram)
    string num; // 24b + 8 in for 

    while(getline(stream, num, ';')){
        num_arr.insert(stoi(num)); // 4b + 2 * i
    }
    for(int i : num_arr){
        cout << i << " ";// 2 * i
    }
    // all ram weight (4b + 2b * i) + (2b * i) + (24b + 368b) + (24b + 2b * i) + 472b = 892 + i * 6 = 60892b = 50 mb(занимает не сразу а в общем)      
    return 0; 
}