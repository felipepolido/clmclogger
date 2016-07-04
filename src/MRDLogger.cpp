#include "mrdplot/MRDLogger.h"
#include <fstream>
#include <iostream>
#include <sstream>

size_t MRDLogger::maxSize() const { return _maxChannelLength - 1; }

MRDLogger::MRDLogger(const unsigned int maxChannelLength, const bool ringBuffer):
    _maxChannelLength(maxChannelLength),
    _ringBuffer(ringBuffer),
    _verbosity(V_NONE)
{
  _reset();
}

MRDLogger::~MRDLogger()
{

}

void MRDLogger::_reset()
{
  _ptStart = 0;
  _ptStartLast = 0;
  _ptEnd = 0;
  _ptEndLast = 0;
  _freq = 1;
  _wrapping = false;
}

bool MRDLogger::_addChannel(const std::string &name, const std::string &unit, const void *ptr, LoggerDataType type)
{
  if (_channels.find(name) != _channels.end())
    return false;

  _channels[name] = DataChannel();

  _channels[name].name = name;
  _channels[name].unit = unit;
  _channels[name].type = type;
  _channels[name].channel = _channels.size()-1;
  _channels[name].ptr = ptr;
  _channels[name].data.resize(_maxChannelLength, 0);

  _outputOrder.push_back(&_channels[name]);
  //std::cout << _channels.size() << " " << _outputOrder.size() << std::endl;

  return true;
}

bool MRDLogger::readFromFile(const std::string &name, const std::string &filePath)
{
  std::ifstream in;
  in.exceptions(std::ifstream::failbit | std::ifstream::badbit);
  
  size_t n_channels, n_points, tot;
  float tmp_data;
  char *ptr = (char *)&tmp_data;

  try {
    _reset();
    std::stringstream ss;

    ss << filePath << name << ".mrd";
    if (_verbosity <= V_INFO) std::cout << "Opening file " << ss.str() << std::endl; 

    in.open(ss.str().c_str());
    // read header
    in >> tot;
    in >> n_channels;
    in >> n_points;
    in >> _freq;

    std::vector<std::string> names(n_channels);
    std::vector<std::string> units(n_channels);

    // read the channel names and units
    std::string tmp;
    for (size_t i = 0; i < n_channels; i++) {
      in >> tmp;
      names[i] = tmp;
      in >> tmp;
      units[i] = tmp;
    }

    // get 3 \n
    in.get();
    in.get();
    in.get();

    // read data
    for (size_t t = 0; t < n_points; t++) {
      for (size_t i = 0; i < n_channels; i++) {
        ptr[3] = in.get();
        ptr[2] = in.get();
        ptr[1] = in.get();
        ptr[0] = in.get();

        auto it = _channels.find(names[i]);
        if (it != _channels.end()) {
          if (_verbosity <= V_DEBUG) std::cout << "read: " << it->first << " " << tmp_data << std::endl;
          it->second.data[_ptEnd] = tmp_data;
        }
      }

      _ptEndLast = _ptEnd;
      _ptStartLast = _ptStart;
      if (_ptEnd >= _maxChannelLength -1) {
        _ptEnd = 0;
        _wrapping = true;
      } else {
        _ptEnd++;
      }
      if (_ptEnd == _ptStart)
        _ptStart = (_ptStart+1) % _maxChannelLength;

      if (_verbosity <= V_DEBUG)
      {
        std::cout << "read: " << tmp_data << std::endl;
        std::cout << "start idx: " << _ptStart << " end idx: " << _ptEnd << std::endl;
      }
    }
    _ptEnd = _ptEndLast;
    _ptStart = _ptStartLast;
  }
  catch (std::ifstream::failure e) {
    std::cerr << "error when parsing data\n";
    return false;
  }

  return true;
}

bool MRDLogger::writeToFile(const std::string &name, const std::string &filePath, const bool useTimeStamp) const
{
  if (size() == 0) {
    return true;
  }

  std::ofstream out;
  out.exceptions(std::ifstream::failbit | std::ifstream::badbit);
  
  try {
    std::stringstream ss;

    //If requested, write log name with date and time stamp:
    std::string timeString = "";
    if (useTimeStamp) {
      char timeBuffer[1000];
      time_t now = time(0);
      struct tm tstruct;
      tstruct = *localtime(&now);
      strftime(timeBuffer, sizeof(timeBuffer), "_%y_%m_%d_%H_%M_%S", &tstruct);
      timeString = timeBuffer;
    }

    ss << filePath << name << timeString << ".mrd";

    if (_verbosity <= V_INFO) std::cout << "Saving to file  " << ss.str() << std::endl; 
    out.open(ss.str().c_str(), std::ofstream::out);
    // write header
    out << this->size()*_channels.size() << " " << _channels.size() << " " << this->size() << " " << _freq << std::endl;

    // write names and units
    for (auto it = _outputOrder.begin(); it != _outputOrder.end(); it++) {
      out << (*it)->name << " " << (*it)->unit << std::endl;
    }

    // write 2 empty line
    out << std::endl << std::endl;

    // write data
    size_t i = _ptStart;
    size_t len = 0;
    while (true) {
      for (auto it = _outputOrder.begin(); it != _outputOrder.end(); it++) {
        len = (*it)->data.size();
        char *ptr = (char *)(&((*it)->data.at(i)));
        out.put(ptr[3]);
        out.put(ptr[2]);
        out.put(ptr[1]);
        out.put(ptr[0]);        
      }
      if (i == _ptEndLast) {
        //If reached end of log, stop
        break;
      } else if (i == len - 1){
        //If reached end of buffer, wrap to start
        i = 0;
      } else {
        i++;
      }
    }

    out.close();
    if (_verbosity <= V_INFO) std::cout << "Finished saving " << ss.str() << std::endl; 

  }
  catch (std::ofstream::failure e) {
    std::cerr << "error when writing data\n";
  }

  return true;
}

size_t MRDLogger::size() const
{
  if (_wrapping) {
    return _maxChannelLength;
  } else {
    return _ptEnd;
  }
}

void MRDLogger::saveData()
{

  if ((_ptEndLast >= _maxChannelLength -1 ) && _ringBuffer == false) {
    //Reached buffer limit, and not wrapping around
    if (_verbosity <= V_WARNING)
      std::cout << "Maximum Channel Lenght reached. Not saving..." << std::endl;
    return;
  }

  //Handle start pointer:
  if (_wrapping) {
    if (_ptStart >= _maxChannelLength - 1) {
      //_ptStart reached buffer limit, wrap around
      _ptStart = 0;
    } else {
      _ptStart++;
    }
  }


  for (auto it = _channels.begin(); it != _channels.end(); it++) {
    switch (it->second.type) {
      case LOGGER_DATA_TYPE_BOOL:
        it->second.data[_ptEnd] = (float) *(bool *)(it->second.ptr);
        break;
      case LOGGER_DATA_TYPE_CHAR:
        it->second.data[_ptEnd] = (float) *(char *)(it->second.ptr);
        break;
      case LOGGER_DATA_TYPE_INT:
        it->second.data[_ptEnd] = (float) *(int *)(it->second.ptr);
        break;
      case LOGGER_DATA_TYPE_FLOAT:
        it->second.data[_ptEnd] = (float) *(float *)(it->second.ptr);
        break;
      case LOGGER_DATA_TYPE_DOUBLE:
        it->second.data[_ptEnd] = (float) *(double *)(it->second.ptr);
        break;
      case LOGGER_DATA_TYPE_LONG:
        it->second.data[_ptEnd] = (float) *(long *)(it->second.ptr);
        break;
      case LOGGER_DATA_TYPE_UCHAR:
        it->second.data[_ptEnd] = (float) *(unsigned char *)(it->second.ptr);
        break;
      case LOGGER_DATA_TYPE_UINT:
        it->second.data[_ptEnd] = (float) *(unsigned int *)(it->second.ptr);
        break;
      case LOGGER_DATA_TYPE_ULONG:
        it->second.data[_ptEnd] = (float) *(unsigned long *)(it->second.ptr);
        break;

      default:
        continue;
    }
  }

  //Handle end pointer
  //_ptEndLast is always one behind _ptEnd
  _ptEndLast = _ptEnd;
  if ((_ptEnd >= _maxChannelLength - 1) && _ringBuffer == true) {
    //Reached buffer limit, wrap around
    //std::cout << "Wrap around" << std::endl;
    _ptEnd = 0;
    _wrapping = true;
  } else {
    _ptEnd++;
  }
}

bool MRDLogger::hasMoreData() 
{
  if (_ptStartLast == _ptEnd) {
    return false;
  } else {
    return true;
  }
}

void MRDLogger::popData()
{
  if (!hasMoreData())
    return;

  for (auto it = _channels.begin(); it != _channels.end(); it++) {
    switch (it->second.type) {
      case LOGGER_DATA_TYPE_BOOL:
        *(bool *)(it->second.ptr) = (bool)it->second.data[_ptStart];
        break;
      case LOGGER_DATA_TYPE_CHAR:
        *(char *)(it->second.ptr) = (char)it->second.data[_ptStart];
        break;
      case LOGGER_DATA_TYPE_INT:
        *(int *)(it->second.ptr) = (int)it->second.data[_ptStart];
        break;
      case LOGGER_DATA_TYPE_FLOAT:
        *(float *)(it->second.ptr) = (float)it->second.data[_ptStart];
        break;
      case LOGGER_DATA_TYPE_DOUBLE:
        *(double *)(it->second.ptr) = (double)it->second.data[_ptStart];
        break;
      case LOGGER_DATA_TYPE_LONG:
        *(long *)(it->second.ptr) = (long)it->second.data[_ptStart];
        break;
      case LOGGER_DATA_TYPE_UCHAR:
        *(unsigned char *)(it->second.ptr) = (unsigned char)it->second.data[_ptStart];
        break;
      case LOGGER_DATA_TYPE_UINT:
        *(unsigned int *)(it->second.ptr) = (unsigned int)it->second.data[_ptStart];
        break;
      case LOGGER_DATA_TYPE_ULONG:
        *(unsigned long *)(it->second.ptr) = (unsigned long)it->second.data[_ptStart];
        break;

      default:
        continue;
    }
  }

  _ptStartLast = _ptStart;
  if (_wrapping) {
    if (_ptStart >= _maxChannelLength -1) {
      _ptStart = 0;
    } else {
      _ptStart++;
    }
  } else {
    _ptStart++;
  }
}

void MRDLogger::setVerbosityLevel(VerbosityType verb)
{
  _verbosity = verb;
  if (_verbosity <= V_DEBUG) {
    std::cout << "Verbotisy level set to : " << _verbosity << std::endl; 
    std::cout << "MRDLogger state: " << std::endl; 
    std::cout << "Maximum channel length is " << _maxChannelLength << std::endl; 
    if (_ringBuffer) std::cout << "Ring buffer flag is true" << std::endl;
  }
}

 