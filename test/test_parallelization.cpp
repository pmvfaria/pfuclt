#include "particle/particle.hpp"
#include "thirdparty/stopwatch/StopWatch.h"

#include <iostream>

int main(int argc, char **argv){

  ::pfuclt::particle::Particles particles(10000, 5, 2);

  std::cout << "Warming up..." << std::endl;

  for(uint i=0; i<1000; ++i)
    particles.normalizeWeights();

  std::cout << "Warmed up!" << std::endl;

  StopWatch watch;
  for(uint i=0; i<10000; ++i)
    particles.normalizeWeights();

  std::cout << "Without parallel library - took " << watch.ElapsedMs() << " ms" << std::endl;

  watch.Restart();
  for(uint i=0; i<10000; ++i)
    particles.normalizeWeights(__gnu_parallel::parallel_balanced);

  std::cout << "Parallel (balanced) - took " << watch.ElapsedMs() << " ms" << std::endl;

  watch.Restart();
  for(uint i=0; i<10000; ++i)
    particles.normalizeWeights(__gnu_parallel::parallel_unbalanced);

  std::cout << "Parallel (unbalanced) - took " << watch.ElapsedMs() << " ms" << std::endl;

  return EXIT_SUCCESS;
}
