#include <iostream>
#include <fstream>
#include<vector>
#include <algorithm>

using namespace std;

int main(){
    // ifstream inf("/home/liuhy/workspace/pcl/test1/datalines.txt");
    // ofstream ouf("/home/liuhy/workspace/pcl/test1/line_3D.txt");
    // double x,y,z,start_x,start_y,start_z,current_x,current_y,current_z;
    // int r,g,b;
    // int idx;
    // int current = -1;
    // if(inf.is_open())
    // {
    //     while(!inf.eof())
    //     {
    //         inf >>x >>y>>z>>r>>g>>b>>idx;
    //         if(current !=idx) {
    //             ouf << start_x <<"    " << start_y << "    "<<start_z<<"    "<<current_x << "    " <<current_y << "    " <<current_z<<"\n";
    //             start_x = x;
    //             start_y = y;
    //             start_z = z;
    //             current = idx;
    //         }
    //         current_x = x;
    //         current_y = y;
    //         current_z = z;
    //     }
    // }
    string s = "12221";
    for(int i = m-1; i > 0 ;i--){
        ans = s[i]-'0';
        int flag =0;
        int sum =0;
        int count =0;
        if(ans % 2 != 0 )
        {
            break;
        }
        for(int j=i-1; j > 0 ; j--)
        {
            if(flag ==1)
            {
                sum += ans;
                ans = 0;
                flag = 0;
            }
            ans = ans + (s[j]-'0')*10;
            flag =1;
            if(!(ans+sum)%11){
                count ++;
            }
        }
    }
    cout << count <<endl;


}

