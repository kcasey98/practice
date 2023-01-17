#include <fstream>
#include <iostream>
using namespace std;

int factorial(int n) {
    /* Base Case: 0! = 1. */
    if (n == 0) {
        return 1;
    }
    /* Recursive Case: n! = n * (n - 1)!. */
    else {
        return n * factorial(n - 1);
    }
}

int main() {
  ifstream f;
  f.open("maze1.txt");
  if (f.is_open()) {
    string line;
    while (getline(f,line))
      cout << line << endl;
    f.close();
  } else {
    cerr << "ERROR" << endl;
  }
  return 0;
}


