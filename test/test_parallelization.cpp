#include <random>
#include <iostream>
#include <algorithm>
#include <parallel/algorithm>

#include "thirdparty/stopwatch/StopWatch.h"
#include "particle/particles.hpp"

::pfuclt::particle::Particles getParticles(){
  ::pfuclt::particle::Particles particles(10000, 5, 2);
  particles.weights.assign(particles.num_particles, 1.0);

  return particles;
}

void warmUp() {
  std::cout << "Warming up..." << std::endl;
  auto particles = getParticles();
  for(uint i=0; i<2000; ++i)
    particles.normalizeWeights(__gnu_parallel::parallel_balanced);

  std::cout << "Warmed up!" << std::endl;
}

void timeNormalizeWeights() {
  auto particles = getParticles();
  StopWatch watch;

  for(uint i=0; i<10000; ++i) {
    try {
      particles.normalizeWeights(std::nullopt);
    }
    catch (const std::range_error &err){
      std::cout<<"caught"<<std::endl;
    }
  }
  std::cout << "NORMALIZE Serial - took " << watch.ElapsedMs() << " ms" << std::endl;

  watch.Restart();
  for(uint i=0; i<10000; ++i) {
    try {
      particles.normalizeWeights(__gnu_parallel::parallel_balanced);
    }
    catch (const std::range_error &err){
      std::cout<<"caught"<<std::endl;
    }
  }
  std::cout << "NORMALIZE Parallel (balanced) - took " << watch.ElapsedMs() << " ms" << std::endl;

  watch.Restart();
  for(uint i=0; i<10000; ++i) {
    try {
      particles.normalizeWeights(__gnu_parallel::parallel_unbalanced);
    }
    catch (const std::range_error &err){
      std::cout<<"caught"<<std::endl;
    }
  }
  std::cout << "NORMALIZE Parallel (unbalanced) - took " << watch.ElapsedMs() << " ms" << std::endl;
}

void timeResize() {
  auto particles = getParticles();
  StopWatch watch;

  std::vector<size_t> resize_numbers(1000000);
  __gnu_parallel::generate(resize_numbers.begin(), resize_numbers.end(), [n = 0]() mutable { return n++; });

  unsigned int i=0;
  watch.Restart();
  for(const auto& num: resize_numbers) {
    particles.resize(num);
    ++i;
  }
  std::reverse(resize_numbers.begin(), resize_numbers.end());
  for(const auto& num: resize_numbers){
    particles.resize(num);
    ++i;
  }

  std::cout << "RESIZE (x" << i << ") Parallel - took " << watch.ElapsedMs() << " ms" << std::endl;
}

int main() {

  warmUp();
  timeNormalizeWeights();
  timeResize();

  return EXIT_SUCCESS;
}
