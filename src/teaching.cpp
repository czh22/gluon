#include "../include/teaching.h"
#include <iostream>

Recorder::Recorder(std::string save_path, std::string file_name, float frequency)
{
  _saved_file_name = file_name;
  _recording_data.append(std::to_string(frequency));
  _recording_data.append("\n");
  _save_path = save_path;
};

void Recorder::writeData(std::vector<double> data)
{
  for (int i = 0; i < 6; i++)
  {
      _recording_data.append(std::to_string(data[i]));
      _recording_data.append(" ");
  }
  _recording_data.append("\n");
};

void Recorder::endRecording()
{
  std::string saving_path;
  saving_path = _save_path;
  saving_path.append("/");
  saving_path.append(_saved_file_name);
  saving_path.append(".txt");

  std::ofstream out(saving_path);
  if (out.is_open())
  {
    out << _recording_data;
    std::cout <<_saved_file_name<<" saved!"<< std::endl;
    out.close();
    _recording_data.clear();
  }
};

Player::Player(std::string save_path, std::string file_name)
{
  std::string data_path;
  data_path = save_path;
  data_path.append("/");
  data_path.append(file_name);
  data_path.append(".txt");
  _record_stream.open(data_path, std::ios::in);
  _record_stream >> _frequency;
};

float Player::getFrequency()
{
  return _frequency;
}

std::vector<double> Player::getData()
{
  for(int i=0; i<6; i++)
  {
    _record_stream >> _top_data[i];
  }
  return _top_data;
};

bool Player::isEnd()
{
  if (_record_stream.eof())
  {
    _record_stream.close();
    return true;
  }
  else
  {
    return false;
  }
};