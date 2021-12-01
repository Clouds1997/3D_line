/*
	DBSCAN Algorithm
	15S103182
	Ethan
*/
#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <ctime>
#include <cstdlib>
#include <limits>
#include <cmath>
#include <stack>
using namespace std;
class point{
public:
	float x;
	float y;
	int cluster=0;
	int pointType=1;//1 noise 2 border 3 core
	int pts=0;//points in MinPts 
	vector<int> corepts;
	int visited = 0;
	point (){}
	point (float a,float b,int c){
		x = a;
		y = b;
		cluster = c;
	}
};
float stringToFloat(string i){
	stringstream sf;
	float score=0;
	sf<<i;
	sf>>score;
	return score;
}
vector<point> openFile(const char* dataset){
	fstream file;
	file.open(dataset,ios::in);
	if(!file) 
    {
        cout <<"Open File Failed!" <<endl;
        vector<point> a;
        return a;
    } 
	vector<point> data;
	int i=1;
	while(!file.eof()){
		string temp;
		file>>temp;
		int split = temp.find(',',0);
		point p(stringToFloat(temp.substr(0,split)),stringToFloat(temp.substr(split+1,temp.length()-1)),i++);
		data.push_back(p);
	}
	file.close();
	cout<<"successful!"<<endl;
	return data;
}
float squareDistance(point a,point b){
	return sqrt((a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y));
}
void DBSCAN(vector<point> dataset,float Eps,int MinPts){
	int len = dataset.size();
	//calculate pts
	cout<<"calculate pts"<<endl;
	for(int i=0;i<len;i++){
		for(int j=i+1;j<len;j++){
			if(squareDistance(dataset[i],dataset[j])<Eps)
				dataset[i].pts++;
				dataset[j].pts++;
		}
	}
	//core point 
	cout<<"core point "<<endl;
	vector<point> corePoint;
	for(int i=0;i<len;i++){
		if(dataset[i].pts>=MinPts) {
			dataset[i].pointType = 3;
			corePoint.push_back(dataset[i]);
		}
	}
	cout<<"joint core point"<<endl;
	//joint core point
	for(int i=0;i<corePoint.size();i++){
		for(int j=i+1;j<corePoint.size();j++){
			if(squareDistance(corePoint[i],corePoint[j])<Eps){
				corePoint[i].corepts.push_back(j);
				corePoint[j].corepts.push_back(i);
			}
		}
	}
	for(int i=0;i<corePoint.size();i++){
		stack<point*> ps;
		if(corePoint[i].visited == 1) continue;
		ps.push(&corePoint[i]);
		point *v;
		while(!ps.empty()){
			v = ps.top();
			v->visited = 1;
			ps.pop();
			for(int j=0;j<v->corepts.size();j++){
				if(corePoint[v->corepts[j]].visited==1) continue;
				corePoint[v->corepts[j]].cluster = corePoint[i].cluster;
				corePoint[v->corepts[j]].visited = 1;
				ps.push(&corePoint[v->corepts[j]]);				
			}
		}		
	}
	cout<<"border point,joint border point to core point"<<endl;
	//border point,joint border point to core point
	for(int i=0;i<len;i++){
		if(dataset[i].pointType==3) continue;
		for(int j=0;j<corePoint.size();j++){
			if(squareDistance(dataset[i],corePoint[j])<Eps) {
				dataset[i].pointType = 2;
				dataset[i].cluster = corePoint[j].cluster;
				break;
			}
		}
	}
	cout<<"output"<<endl;
	//output
	fstream clustering;
	clustering.open("clustering.txt",ios::out);
	for(int i=0;i<len;i++){
		if(dataset[i].pointType == 2)
			clustering<<dataset[i].x<<","<<dataset[i].y<<","<<dataset[i].cluster<<"\n";
	}
	for(int i=0;i<corePoint.size();i++){
			clustering<<corePoint[i].x<<","<<corePoint[i].y<<","<<corePoint[i].cluster<<"\n";
	}
	clustering.close();
}
int main(int argc, char** argv) {
	vector<point> dataset = openFile("dataset3.txt");
	DBSCAN(dataset,1.5,2);
	return 0;
}