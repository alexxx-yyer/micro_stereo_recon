#include "DirTools.h"
#include "opencv2/core.hpp"
#include <cstdarg>

using namespace std;
using namespace cv;
using fs = FileStorage;
using fn = FileNode;
using fnit = FileNodeIterator;
namespace fsys = filesystem;



bool isImgFile(fsys::path path) {
	return isFileType(path, { ".png", ".jpg" });
}

bool isPGMFile(fsys::path path) {
	return isFileType(path, { ".pgm" });
}

bool isFileType(fsys::path path, vector<string> list) {
	string exten = path.extension().string();
	for (string type : list) {
		if (type == exten) {
			return true;
		}
	}
	return false;
}

bool getImgList(const std::filesystem::path _path, std::vector<std::string>& list) {
	if (fsys::is_directory(_path)) {
		return getImgList_Dir(_path, list);
	}
	else if (isFileType(_path, { ".yml", ".yaml", ".xml" })) {
		return loadImgList_YML(_path, list);
	}
	else if (isImgFile(_path)) {
		list.push_back(_path.string());
		return true;
	}
	return false;
}

bool getImgList_Dir(const fsys::path dir, vector<string>& list) {
	if (!fsys::exists(dir)) {
		printf("Invalid directory path.\n");
		return false;
	}
	fsys::directory_iterator it(dir);
	for (fsys::directory_entry entry : it) {
		if (isImgFile(entry.path())) {
			list.push_back(entry.path().string());
		}
	}
	return list.size() != 0;
}

bool loadImgList_YML(const fsys::path _path, vector<string>& list) {
	if (!fsys::exists(_path)) {
		printf("Invalid directory path.\n");
		return false;
	}
	fs file(_path.string(), fs::READ);
	fn node = file.getFirstTopLevelNode();
	fnit it = node.begin();
	int count = 0;
	for (; it != node.end(); it++) {
		list.push_back(*it);
	}
	return true;
}

bool getPGMList(const fsys::path _path, vector<string>& list) {
	if (!fsys::exists(_path)) {
		printf("Invalid directory path.\n");
		return false;
	}
	fsys::directory_iterator it(_path);
	for (fsys::directory_entry entry : it) {
		if (isPGMFile(entry.path())) {
			list.push_back(entry.path().string());
		}
	}
	return list.size() != 0;
}

bool getFileList(const fsys::path _path, vector<string>& list) {
	if (!fsys::exists(_path)) {
		printf("Invalid directory path.\n");
		return false;
	}
	fsys::directory_iterator it(_path);
	for (fsys::directory_entry entry : it) {
		list.push_back(entry.path().string());
	}
	return list.size() != 0;
}

bool loadImgPair_YML(const std::filesystem::path _path, std::vector<std::string>& lImgs, std::vector<std::string>& rImgs) {
	if (!fsys::exists(_path)) {
		printf("No such file: %s\n", _path.string().c_str());
		return false;
	}
	fs file(_path.string(), fs::READ);
	fn node = file.getFirstTopLevelNode();
	fnit it = node.begin();
	int count = 0;
	for (; it != node.end(); it++) {
		if (count % 2)
			rImgs.push_back(*it);
		else
			lImgs.push_back(*it);
		count++;
	}
	if (lImgs.size() != rImgs.size()) {
		printf("lists have not aligned\n");
		return false;
	}
	return true;
}

bool getImgList_Dir(const std::string dir, std::vector<std::string>& list)
{
	fsys::path _dir(dir);
	return getImgList_Dir(_dir, list);
}

bool getImgList_File(const std::string file, std::vector<std::string>& list)
{
	fsys::path _file(file);
	if(isFileType(_file, {".yml", ".xml", ".yaml"}))
		return loadImgList_YML(_file, list);
	else if(isFileType(_file, {".png", ".jpg", ".pgm"}))
	{
		list.push_back(file);
		return true;
	}
	else
	{
		return false;
	}
}