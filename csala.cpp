// kidan jin, july 29 2019, main algorithm part
#define PY_SSIZE_T_CLEAN
#include <Python.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>
#include <ctime>

#define EXP 2.718282
#define PI 3.141593

using namespace std;

/* please insert walls to wallInfo(line 539) if you need */

//---------- [Global Variables] ------------------------------------------------------------------------------

// centroid 계산 시 선택할 점의 개수 (10으로 고정시킨 상태)
int selectHighRssiNum = 10;	/* 동적으로 바뀌면 좋을 거 같음 */

// table에 포함할 점 개수
int TableColumnSize = selectHighRssiNum;	

// 각 점마다 선택할 주변 점의 개수
const int NEARSelectNum = 10;	/*이것도 동적으로 바뀌도록 고려해보면 좋을 것 같음*/

// room, grid cell
const int Nrow = 800, Ncol = 800;		// 방 크기
const int GridCellSize = 10;			// grid cell 크기(정밀도)
const int nrow = Nrow/GridCellSize;		// grid 행 개수
const int ncol = Ncol/GridCellSize;		// grid 열 개수

// 벽 근처 판정 거리
const int NearWallDistance = 100;		



//---------- [Objects Declaration] ----------------------------------------------------------------------------

// position을 나타내는 객체
class position
{
private:
	double x;
	double y;

public:
	position(double x, double y) : x(x), y(y) {}

	position(const position& copy) : x(copy.x), y(copy.y) {}

	double getX() const { return x; }

	double getY() const { return y; }
};


// position의 information을 나타내는 객체
class Info : public position
{
private:
	long int timestamp;
	int rssi;

public:
	Info(double x, double y, long int t, int r)
		: position(x, y), timestamp(t), rssi(r) {}

	Info(const Info& copy)
		: position(copy), timestamp(copy.timestamp), rssi(copy.rssi) {}

	long int getTimestamp() const { return timestamp; }

	position getPosition() const
	{
		position p(this->getX(), this->getY());
		return p;
	}

	int getRssi() const { return rssi; }
};


// power-distance Table의 한 열을 표현하는 객체
class TableCol : public position
{
private:
	int w;
	double l;
	double u;
	double sigma;

public:
	TableCol(double x, double y, int rssi, double distance, double avg, double std)
	:position(x, y), w(rssi), l(distance), u(avg), sigma(std) {}

	TableCol(const TableCol& copy)
	:position(copy), w(copy.w), l(copy.l), u(copy.u), sigma(copy.sigma) {}

	int getRssi() const { return w; }

	double getDistance() const { return l; }

	double getAvg() const { return u; }

	double getStd() const { return sigma; }
	
	position getPosition() const
	{
		position p(this->getX(), this->getY());
		return p;
	}
};


// 방의 벽을 표현하는 객체
enum wallType {horizontal = 0, vertical};

class Wall
{
private:
	enum wallType wt;
	double fix;
	double start;
	double end;
	
public:
	Wall(enum wallType wt, double f, double s, double e)
	:wt(wt), fix(f), start(s), end(e) {}

	Wall(const Wall& copy)
	:wt(copy.wt), fix(copy.fix), start(copy.start), end(copy.end) {}
	
	enum wallType getWallType() const { return wt; }
	
	double getFix() const { return fix; }
	
	double getStart() const { return start; }
	
	double getEnd() const { return end; }	
};



//---------- 1. RSSI-Centroid Estimation --------------------------------------------------

// Ci를 rssi 기준 내림차순 정렬하는 함수
bool CompareRssi(Info& d1, Info& d2) {
	return d1.getRssi() > d2.getRssi();
}

// Ci를 받아 정렬해서 Ci*을 만들고, 선택할 점의 개수 m을 받아 조절하고, centroid p*를 구해 반환하는 함수
position CentroidEstimation(vector<Info>& Ci)
{		
	/* const int n = Ci.size();	// n: Ci size(# of all point)	*/
	int m = selectHighRssiNum;
	
	// Ci rssi순 정렬
	sort(Ci.begin(), Ci.end(), CompareRssi);

/*	// m값 조절
	int rssi_m = Ci[m - 1].getRssi();		// m번째 자료의 rssi값

	while (m < n)							// rssi가 같지 않을때까지 반복
	{
		int rssi_now = Ci[m].getRssi();		// m+1번째 자료의 rssi값
		if (rssi_now == rssi_m)				// 현재 자료와 이전 자료 값이 비교해 같으면 m 값 증가
			m++;							
		else
			break;
	}										*/
	
	// centroid 계산, 반환
	double xstar = 0, ystar = 0;

	for (int i = 0; i < m; i++)
	{
		xstar += Ci[i].getX()/m;
		ystar += Ci[i].getY()/m;
	}

	position pstar(xstar, ystar);
	return pstar;
}



//---------- 2. Wall-Corner Handling ------------------------------------------------------------------------------------

// wallInfo를 정렬하는 함수
bool wallsort(Wall& w1, Wall& w2) {
	if (w1.getWallType() < w2.getWallType())
		return true;
	else if ((w1.getWallType() == w2.getWallType()) && (w1.getFix() < w2.getFix()))
		return true;
	else
		return false;
}

// centroid's types
enum centroidType{OPENSPACE = 0, WALL, CORNER};

// wall info를 통해 centroid type을 판단하여 p*을 조정하고, TYPE을 반환하는 함수
enum centroidType WallCornerHandlig(const vector<Info>& Ciprime, vector<Wall>& wallinfo, position& centroid)
{
	//sorting wall Info
	sort(wallinfo.begin(), wallinfo.end(), wallsort);
		
	// get # of all wall and horizontal walls
	int wallcount = wallinfo.size();
	int hwcount = 0;
	while(wallinfo[hwcount].getWallType() == 0)
		hwcount++;
	
	// get centroid coordinate
	double cenX = centroid.getX();
	double cenY = centroid.getY();	
	
	// temporary vector for handling (원래 Ci를 보존하기 위함)
	vector <Info> newCi;
			
	// check centroid is near horizontal wall
	bool isnearHWall = false;
	int hindex;	
	for(hindex = 0; hindex < hwcount; hindex++)
	{
		// check centroid y-coordinate is between wall start and end, and centroid x-coordinate is near the wall
		if((cenY > wallinfo[hindex].getStart()) && (cenY < wallinfo[hindex].getEnd()) && (abs(cenX-wallinfo[hindex].getFix()) < NearWallDistance))
		{
			isnearHWall = true;
			break;
		}
	}
	
	// check centroid is near vertical wall
	bool isnearVWall = false;
	int vindex;	
	for(vindex = hwcount; vindex < wallcount; vindex++)
	{
		// check centroid x-coordinate is between wall start and end, and centroid y-coordinate is near the wall
		if((cenX > wallinfo[vindex].getStart()) && (cenX < wallinfo[vindex].getEnd()) && (abs(cenY-wallinfo[vindex].getFix()) < NearWallDistance))
		{
			isnearVWall = true;
			break;
		}
	}
	
	//for check
	cout<<boolalpha;
	cout<<"\tnear horizontal wall? "<<isnearHWall<<endl;
	if(isnearHWall){
		cout<<" -> centroid is near the horizontal wall from ("<<wallinfo[hindex].getFix()<<", "<<wallinfo[hindex].getStart();
		cout<<") to ("<<wallinfo[hindex].getFix()<<", "<<wallinfo[hindex].getEnd()<<')'<<endl;
	}
	cout<<"\tnear vertical wall? "<<isnearVWall<<endl;
	if(isnearVWall){
		cout<<" -> centroid is near the vertical wall from ("<<wallinfo[vindex].getStart()<<", "<<wallinfo[vindex].getFix();
		cout<<") to ("<<wallinfo[vindex].getEnd()<<", "<<wallinfo[vindex].getFix()<<')'<<endl;
	}	
	
	// corner handling
	if(isnearHWall*isnearVWall)		
	{
		double corX = wallinfo[hindex].getFix();		// get coordinate of corner
		double corY = wallinfo[vindex].getFix();	
		
		for(int i = 0; i < selectHighRssiNum; i++)		// copy Ci' and imaginary symmetry points to newCi
		{			
			newCi.push_back(Ciprime[i]);
			Info tmp1(2*corX-Ciprime[i].getX(), Ciprime[i].getY(), Ciprime[i].getTimestamp(), Ciprime[i].getRssi());
			newCi.push_back(tmp1);
			Info tmp2(Ciprime[i].getX(), 2*corY-Ciprime[i].getY(), Ciprime[i].getTimestamp(), Ciprime[i].getRssi());
			newCi.push_back(tmp2);
			Info tmp3(2*corX-Ciprime[i].getX(), 2*corY-Ciprime[i].getY(), Ciprime[i].getTimestamp(), Ciprime[i].getRssi());
			newCi.push_back(tmp3);
		}
		
		centroid = CentroidEstimation(newCi);
		
		return CORNER;
	}
	// horizontal wall handling	
	else if(isnearHWall)
	{
		double wallX = wallinfo[hindex].getFix();		
		
		for(int i = 0; i < selectHighRssiNum; i++)		// copy Ci' and imaginary symmetry points to newCi
		{			
			newCi.push_back(Ciprime[i]);
			Info tmp(2*wallX-Ciprime[i].getX(), Ciprime[i].getY(), Ciprime[i].getTimestamp(), Ciprime[i].getRssi());
			newCi.push_back(tmp);							
		}

		centroid = CentroidEstimation(newCi);
		
		return WALL;
	}
	// vertical wall handling	
	else if(isnearVWall)		
	{
		double wallY = wallinfo[vindex].getFix();	

		for(int i = 0; i < selectHighRssiNum; i++)		// copy Ci' and imaginary symmetry points to newCi
		{			
			newCi.push_back(Ciprime[i]);
			Info tmp(Ciprime[i].getX(), 2*wallY-Ciprime[i].getY(), Ciprime[i].getTimestamp(), Ciprime[i].getRssi());
			newCi.push_back(tmp);							
		}

		centroid = CentroidEstimation(newCi);
		
		return WALL;
	}
	else
		return OPENSPACE;
	
}



//---------- 3. Power-Distance Table Construction ------------------------------------------------------------------------

//두 점 사이의 거리를 구하는 함수
double getDistance(const position p1, const position p2) {

	double distance;
	distance = sqrt(pow((p1.getX() - p2.getX()), 2) + pow((p1.getY() - p2.getY()), 2));

	return distance;
}

// Ci', p*를 받아, Power-Distance Table을 만드는 함수
void PowerDistanceTable(const vector<Info>& Ciprime, vector<TableCol>& pdtable, const position pstar) 
{
	// 테이블 각 행의 값 및 점의 위치를 저장할 열 벡터 선언
	vector<double> x;
	vector<double> y;
	vector<int> w;
	vector<double> l;
	vector<double> u;
	vector<double> sigma;
	const int n = Ciprime.size();	
	
	// Ci'에 저장된 모든 점의 좌표, rssi, p*와의 거리 l을 계산해 각 행별로 저장
	for (int k = 0; k < n; k++)
	{
		position pk = Ciprime[k].getPosition();				
		x.push_back(pk.getX());								
		y.push_back(pk.getY());								
		w.push_back(Ciprime[k].getRssi());					
		l.push_back(getDistance(pstar, pk));				
	}	

	// table에 저장할 각 점에 대해 가까운 점을 full search해서 choice하고, l의 평균과 표준편차를 저장	
	for (int k = 0; k < TableColumnSize; k++)	
	{
		vector<pair<double, int> > LDis;		// Pk와 각 점 사이 distance, index를 담는 pair벡터

		for (int i = 0; i < n; i++)
		{
			position pk(x[k], y[k]);
			position pi(x[i], y[i]);
			LDis.push_back(pair<double, int>((getDistance(pk, pi)), i));
		}

		sort(LDis.begin(), LDis.end());			// LDis vector를 dis, index순으로 오름차순 정렬
	
		double avg = 0.0;						// l의 평균 계산
		
		for (int j = 0; j < NEARSelectNum; j++)				
		{
			avg += l[LDis[j].second]/NEARSelectNum;
		}
		
		u.push_back(avg);

		double std = 0.0;						// l의 표준편차 계산

		for (int j = 0; j < NEARSelectNum; j++)
		{
			std += pow((l[LDis[j].second] - avg), 2);
		}

		std = sqrt(std / NEARSelectNum);
		sigma.push_back(std);

	}

	// 계산 결과 벡터의 각 요소를 뽑아 column객체를 만들고 pdtable에 차곡차곡 push
	for (int k = 0; k < TableColumnSize; k++)
	{
		TableCol col(x[k], y[k], w[k], l[k], u[k], sigma[k]);
		pdtable.push_back(col);
	}
}



//---------- 4. Grid-Weight Map Construction --------------------------------------------------------------------

// Gaussian 확률 계산 함수
double Gaussian(const double x, const double u, const double sigma) {
	double Z = (x-u)/sigma;
	if (Z > 4)					// underflow 방지, Z>4면 확률이 0.01% 미만이어서 의미 없음
		return 0;
	else
		return pow(EXP, -Z*Z/2)/sqrt(2*PI)/sigma;
}

// pdtable을 받아 gridmap을 만들어 추정 실제 위치를 반환하는 함수
position GridWeightMap(vector<vector<double> >& gridmap, const vector<TableCol>& pdtable)
{
	const int pnum = pdtable.size();	// number of points in power distance table
	
	for(int j = 0; j < pnum; j++)						// pnum개의 점에 대해 반복
	{
		// get position of pj, uj, sigmaj, transform to grid map
		position tpj(pdtable[j].getX()/GridCellSize, pdtable[j].getY()/GridCellSize);
		double tuj = pdtable[j].getAvg()/GridCellSize;
		double tsigmaj = pdtable[j].getStd()/GridCellSize;
		for(int a = 0; a < nrow; a++)							// gridmap의 모든 셀에 대해 반복
		{
			for(int b = 0; b < ncol; b++)
			{
				position gab(a, b);
				double dab = getDistance(gab, tpj);				// calc distance between grid cell and pj cell
				double mab = Gaussian(dab, tuj, tsigmaj);		// calc probability
				gridmap[a][b] += mab;							// summation to gridmap
			}
		}
	}		

	// search maximun weight grid cells
	double maxWeight = 0.0;
	vector<position> criticalcell;			// critical cell의 좌표를 담는 벡터
	
	for(int a = 0; a < nrow; a++)							
	{
		for(int b = 0; b < ncol; b++)
		{
			if(gridmap[a][b] > maxWeight)
			{
				maxWeight = gridmap[a][b];
				criticalcell.clear();
				position temp(a, b);
				criticalcell.push_back(temp);					
			}
			else if(gridmap[a][b] == maxWeight)
			{
				position temp(a, b);
				criticalcell.push_back(temp);
			}								
		}
	}
	
	const int cnum = criticalcell.size();	// number of critical cells in grid map

	//for check
	cout<<"\t# of critical cells: "<<cnum<<endl;
	
	// find centroid of critical cells
	double max_a = 0.0, max_b = 0.0;
	for(int i = 0; i < cnum; i++)
	{
		max_a += criticalcell[i].getX()/cnum;
		max_b += criticalcell[i].getY()/cnum;
	}
	
	// 구한 centroid critical cell을 xy평면으로 inverse transform한 점(실제 위치)을반환  
	position pr(max_a*GridCellSize, max_b*GridCellSize);	
	return pr;
}



//------------------------------ SALA MAIN --------------------------------------------------------

static PyObject* csala_sala_algorithms(PyObject* self, PyObject* args) {
		
	PyObject* device, * reports;
	double pos_star_x, pos_star_y;
	double location_x, location_y;
	vector<Info> Ci;

	if (!PyArg_ParseTuple(args, "OO", &device, &reports)) {
		return Py_BuildValue("i", -1);
	}

	// device 변수는 사용되지 않습니다. 디버깅용 변수입니다.
	PyObject_Print(device, stderr, 0);
	fprintf(stderr, "\n");
	
	int report_size = PySequence_Size(reports);		// 받은 데이터의 갯수
	fprintf(stderr, "report size: %d\n", report_size);

	// read Report data, convert to C++ variables, make Ci
 	for (int ri = 0; ri < report_size; ri++) {

		PyObject* report_recode;
		report_recode = PySequence_GetItem(reports, ri);

		PyObject* py_position, * py_rssi, * py_timestamp;
		double sx, sy;
		int rssi;
		long int timestamp;

		py_position = PyObject_GetAttrString(report_recode, "position");
		py_rssi = PyObject_GetAttrString(report_recode, "rssi");
		py_timestamp = PyObject_GetAttrString(report_recode, "timestamp");

		rssi = PyLong_AsLong(py_rssi);
		timestamp = PyLong_AsLong(py_timestamp);
		sx = PyLong_AsLong(PyObject_GetAttrString(py_position, "x"));
		sy = PyLong_AsLong(PyObject_GetAttrString(py_position, "y"));
		
		Info onedata(sx, sy, timestamp, rssi);		// Infomation of a report data

		Ci.push_back(onedata);						// maek array vector Ci with Info form 	
	}

	clock_t start, end;
	double ExecutionTime;
	start = clock();
	
	cout<<endl<<"[SALA] Centroid Estimation"<< endl;
	position star = CentroidEstimation(Ci);		
	cout<<"\testimated centroid: ("<<star.getX()<<", "<<star.getY()<<')'<<endl;
	
	// wall information of room
	vector<Wall> wallInfo;							
	wallInfo.push_back(Wall(horizontal, 0, 0, Ncol));
	wallInfo.push_back(Wall(horizontal, Nrow, 0, Ncol));	
	wallInfo.push_back(Wall(vertical, 0, 0, Nrow));
	wallInfo.push_back(Wall(vertical, Ncol, 0, Nrow));
	/////// please insert walls to wallInfo if you need ///////	
	
	///////////////////////////////////////////////////////////
	
	cout<<"[SALA] Wall Corner Handling"<< endl;
	enum centroidType ctype = WallCornerHandlig(Ci, wallInfo, star);
	cout<<"\tcentroid Type: " << ctype << endl;
	cout<<"\tnew centroid: (" << star.getX() << ", " << star.getY() << ')' << endl;
		
	cout<<"[SALA] Construct Power-Distance Table"<< endl;
	vector<TableCol> table;
	PowerDistanceTable(Ci, table, star);

	cout<<"[SALA] Construct Grid-weight map"<<endl;
	vector<vector<double> > map(nrow, vector<double>(ncol, 0.0));
	position location = GridWeightMap(map, table);
	cout<<"\tIoT location: (" << location.getX() << ", " << location.getY() << ')' << endl;
	
	end = clock();
	ExecutionTime = (double)(end-start) / CLOCKS_PER_SEC;
	cout<<"...Execution time: "<<ExecutionTime<<endl<<endl;
	
	// get location of IoT device
	pos_star_x = star.getX();
	pos_star_y = star.getY();
	location_x = location.getX();
	location_y = location.getY();

	
	return Py_BuildValue("dddd", pos_star_x, pos_star_y, location_x, location_y);
}

//////////////////////////////////////////////////////////////////

static PyMethodDef csalaMethods[] = {

	 {"sala_algorithms", csala_sala_algorithms, METH_VARARGS,
	 "c-SALA wrapper"},

	{NULL, NULL, 0, NULL} /* Sentinel */
};


// 모듈의 문서 상수. 이 모듈이 어떤 모듈인지 텍스트를 통해 설명할 때 사용
static char csala_doc[] = "moudle documentation\n";

// 파이썬 모듈 정의부
static struct PyModuleDef csalamodule = {

	PyModuleDef_HEAD_INIT,
	"csala",	/* name of module */
	csala_doc,	/* module documentation, may be NULL */
	-1,			/* size of per-interpreter state of the module,
				   or -1 if the module keeps state in global variables. */
	csalaMethods	/* 모듈 내의 함수 정의부 */
};


PyMODINIT_FUNC PyInit_csala(void) {

	return PyModule_Create(&csalamodule);
}
