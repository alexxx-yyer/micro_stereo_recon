#include "ImgAnalyzeTools.h"

using namespace std;
using namespace cv;

static void init_areas(vector<float>& areas, float* ptr, int ws) {
	areas.push_back(ptr[0]);
	for (int i = 1; i < ws; i++) {
		areas[0] += ptr[i];
	}
}

// find peaks of all the regions that area in window bigger than certain threshold
vector<int> findPeaks(Mat& hist) {
	vector<int> peaks;
	Mat histn;
	normalize(hist, histn, 1, 0, NORM_INF);
	int ws = 3; // window size, must be odd integer
	vector<float> areas;
	float* ptr = histn.ptr<float>();
	float max = 0.;
	init_areas(areas, ptr, ws);
	for (int i = 0; i < histn.total() - ws; i++) {
		areas.push_back(areas.back() - ptr[i] + ptr[i+ws]);
		if (ptr[i] > max) {
			max = ptr[i];
		}
	}
	for (int i = histn.total() - ws; i < histn.total(); i++) {
		if (ptr[i] > max) {
			max = ptr[i];
		}
	}
	// Ñ°·å
	bool finding = false;
	float threshold = 0.2 * max;
	max = 0.;
	int idx = -1;
	for (int i = 0; i < areas.size(); i++) {
		if (areas[i] > threshold) {
			if (!finding) {
				finding = true;
				for (int j = 0; j < ws; j++) {
					if (ptr[i + j] > max) {
						max = ptr[i + j];
						idx = i + j;
					}
				}
			}
			else {
				if (ptr[i + ws - 1] > max) {
					max = ptr[i + ws - 1];
					idx = i + ws - 1;
				}
			}
		}
		else {
			if (finding) {
				finding = false;
				peaks.push_back(idx);
				max = 0.;
			}
			continue;
		}
	}
	if (finding) {
		finding = false;
		peaks.push_back(idx);
		max = 0.;
	}

	return peaks;
}