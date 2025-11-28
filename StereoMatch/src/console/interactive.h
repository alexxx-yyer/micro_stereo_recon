#pragma once

#include "algorithms.h"

#include "opencv2/imgcodecs.hpp"

enum step {GoNext, GoBack, Exit};

// Abstract Interface for Console Interaction
class AlgConsoleInterface {
public:
    virtual ~AlgConsoleInterface() {delete algorithm_ptr;}
    virtual void showSetting() = 0;
    virtual void modifySetting() = 0;

    std::string getType() {
        return algorithm_ptr->getType();
    }

    Alg_Impl* algorithm_ptr = nullptr;
};

class SGBM_Console : public AlgConsoleInterface {
public:
    SGBM_Console();
    ~SGBM_Console() {};
    void showSetting();
    void modifySetting();
};

class BM_Console : public AlgConsoleInterface {
public:
    BM_Console();
    ~BM_Console() {};
    void showSetting();
    void modifySetting();
};

class ELAS_Console : public AlgConsoleInterface {
public:
    ELAS_Console();
    ~ELAS_Console() {};
    void showSetting();
    void modifySetting();
};

class CommandLineEngine
{
public:
    CommandLineEngine(cv::CommandLineParser &_parser);
    ~CommandLineEngine();
    int run();
    void changeAlg(std::string type);
    AlgConsoleInterface *createAlg(std::string type);

private:
    AlgConsoleInterface *alg;
    cv::CommandLineParser parser;
    std::vector<std::string> lImgs, rImgs;
    cv::Mat Q;

    step process(std::string lpath, std::string rpath);
    bool getImageList(std::vector<std::string> &lImgs, std::vector<std::string> &rImgs);
};
