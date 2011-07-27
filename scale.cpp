#include <algorithm>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <omp.h>
#include <vector>

using namespace std;

// TODO: This program scales the coordinates of an input file?
// By what? A constant factor? Why is this done?

int main (int argc, char** argv) {

	if (argc < 3) {
		cerr << "Usage: ./scale infile outfile\n";
		return -1;
	}

	ifstream infile;
	infile.open(argv[1], ios::in);
	if (!infile.is_open()) {
		cerr << "ERROR: File not found. Exiting...\n";
		return -1;
	}

	vector<double> x, y, z;
    double min[] = {0, 0, 0};
	double max[] = {0, 0, 0};
	double xent, yent, zent;

	while (infile.good()) {
		infile >> xent >> yent >> zent;
		x.push_back(xent); y.push_back(yent); z.push_back(zent);
	}

	min[0] = *min_element(x.begin(), x.end());
	min[1] = *min_element(y.begin(), y.end());
	min[2] = *min_element(z.begin(), z.end());

	cout << setprecision(12) << min[0] << " " << min[1] << " " << min[2] << endl;

	infile.close();

	//Centers all points in each coordinate axis in parallel
	#pragma omp parallel for 
	for (size_t i=0; i < x.size(); ++i) {
		x[i] -= min[0];
		y[i] -= min[1];
		z[i] -= min[2];
	}

	//Obtain maximum values of the centered coordinates
	max[0] = *max_element(x.begin(), x.end());
	max[1] = *max_element(y.begin(), y.end());
	max[2] = *max_element(z.begin(), z.end());

	cout << setprecision(12) << max[0] << " " << max[1] << " " << max[2] << endl;

	//Scales all coordinate axes in parallel
	#pragma omp parallel for 
	for (size_t i=0; i < x.size(); ++i) {
		x[i] = (2*(x[i] / max[0]))-1;
		y[i] = (2*(y[i] / max[1]))-1;
		z[i] = (2*(z[i] / max[2]))-1;
	}

	//write the scaled points to file
	ofstream outfile(argv[2], ios::out);
	for (size_t i = 0; i < x.size(); ++i) 
		outfile << setprecision(12) << x[i] << "  " << y[i] << "  " << z[i] << endl;

	outfile.close();

	return 0;

}
