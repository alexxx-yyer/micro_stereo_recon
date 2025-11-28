#pragma once

#include <filesystem>
#include <vector>
#include <string>

bool isFileType(std::filesystem::path path, std::vector<std::string> list);
bool isImgFile(const std::filesystem::path _path);
bool getFileList(const std::filesystem::path _path, std::vector<std::string>& list);
bool getImgList(const std::filesystem::path _path, std::vector<std::string>& list);
bool getImgList_Dir(const std::filesystem::path dir, std::vector<std::string>& list);
bool getImgList_Dir(const std::string dir, std::vector<std::string>& list);
bool getImgList_File(const std::string _file, std::vector<std::string>& list);
bool loadImgList_YML(const std::filesystem::path _path, std::vector<std::string>& list);
bool getPGMList(const std::filesystem::path _path, std::vector<std::string>& list);
bool loadImgPair_YML(const std::filesystem::path _path, std::vector<std::string>& lImgs, std::vector<std::string>& rImgs);