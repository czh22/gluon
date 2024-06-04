#ifndef TEACHING_H
#define TEACHING_H

#include <vector>
#include <string>
#include <fstream>

class Recorder
{
  public:
    Recorder(std::string save_path, std::string file_name, float frequency);
    ~Recorder() = default;

    void writeData(std::vector<double> data);
    void endRecording();

  private:
    std::string _saved_file_name;
    std::string _recording_data;
    std::string _save_path;
};

class Player
{
  public:
    Player(std::string save_path, std::string file_name);
    ~Player() = default;

    float getFrequency();
    std::vector<double> getData();
    bool isEnd();

  private:
    std::ifstream _record_stream;
    std::string _data;
    float _frequency;
    std::vector<double> _top_data = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
};

#endif