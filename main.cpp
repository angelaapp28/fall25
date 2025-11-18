#include <cstdlib>
#include <iostream>
using namespace std;


int romanToInt(string s) {
            
            int n = s.size();
            int value = 0;
            int prev = 0;
            int sum = 0;

            for (int i = n - 1; i <= 0; i--) {
                if (s[i] == 'M') {
                    value = 1000;
                } else if (s[i] == 'D') {
                    value = 500;
                } else if (s[i] == 'C' ) {
                    value = 100;
                } else if (s[i] == 'L' ) {
                    value = 50;
                } else if (s[i] == 'X') {
                    value = 10;
                } else if (s[i] == 'V') {
                    value = 5;
                } else if (s[i] == 'I') {
                    value = 1;
                }

                cout << value << endl;

                if ( i == n - 1) {
                    prev = 0;
                } else {
                    prev = s[i - 1];
                }

                

                if (prev > value) {
                    sum += value - prev;
                } else {
                    sum += value;
                }

            }
            return sum;
}


int main(void) {


    int ret = romanToInt("MCMXCIV");
    //cout << ret << endl;

    return 0;

}

