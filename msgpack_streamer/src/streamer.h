/**
 * Base implementation of object streaming.
 * Overridden to provide varying implementations.
 */
#ifndef __STREAMER_H__
#define __STREAMER_H__

template<typename T>
class Streamer
{
private:
  
public:
  Streamer(std::string topic) {};
  virtual ~Streamer() {};
  
  virtual pack() {};
  virtual unpack() {};
};

#endif /* __STREAMER_H__ */
