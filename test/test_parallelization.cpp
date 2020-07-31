#include <random>
#include <iostream>
#include <algorithm>
#include <parallel/algorithm>

#include "thirdparty/stopwatch/StopWatch.h"
#include "particle/particles.hpp"


pfuclt::particle::Particles getParticles()
{
  pfuclt::particle::Particles particles(10000, 5, 2);
  std::fill(particles.weights.begin(), particles.weights.end(), 1.0);

  return particles;
}

void warmUp()
{
  std::cout << "Warming up..." << std::endl;
  auto particles = getParticles();
  for (int i = 0; i < 2000; ++i)
    particles.normalizeWeights(__gnu_parallel::parallel_balanced);

  std::cout << "Warmed up!" << std::endl;
}

void timeNormalizeWeights()
{
  auto particles = getParticles();
  StopWatch watch;

  for (int i = 0; i < 10000; ++i) {
    try {
      particles.normalizeWeights(std::nullopt);
    }
    catch (const std::range_error &err) {
      std::cout << "caught" << std::endl;
    }
  }
  std::cout << "NORMALIZE Serial - took " << watch.ElapsedMs() << " ms" << std::endl;

  watch.Restart();
  for (int i = 0; i < 10000; ++i) {
    try {
      particles.normalizeWeights(__gnu_parallel::parallel_balanced);
    }
    catch (const std::range_error &err) {
      std::cout<<"caught"<<std::endl;
    }
  }
  std::cout << "NORMALIZE Parallel (balanced) - took " << watch.ElapsedMs() << " ms" << std::endl;

  watch.Restart();
  for (int i = 0; i < 10000; ++i) {
    try {
      particles.normalizeWeights(__gnu_parallel::parallel_unbalanced);
    }
    catch (const std::range_error &err){
      std::cout<<"caught"<<std::endl;
    }
  }
  std::cout << "NORMALIZE Parallel (unbalanced) - took " << watch.ElapsedMs() << " ms" << std::endl;
}

void timeResize()
{
  auto particles = getParticles();
  StopWatch watch;

  std::vector<size_t> resize_numbers(1000000);
  __gnu_parallel::generate(resize_numbers.begin(), resize_numbers.end(), [n = 0]() mutable { return n++; });

  int i = 0;
  watch.Restart();
  for (const auto& num : resize_numbers) {
    particles.resize(num);
    ++i;
  }
  std::reverse(resize_numbers.begin(), resize_numbers.end());
  for (const auto& num : resize_numbers) {
    particles.resize(num);
    ++i;
  }

  std::cout << "RESIZE (x" << i << ") Parallel - took " << watch.ElapsedMs() << " ms" << std::endl;
}

void timeAlgorithmTarget()
{
  std::default_random_engine generator;
  std::uniform_real_distribution<double> dist(0.0, 1.0);

  auto particles = getParticles();

  StopWatch watch;
  for (uint t = 0; t < particles.targets.size(); ++t) {

    int m_star;
    double max_weight;

    for (int m = 0; m < 10000; ++m) {

      max_weight = -1.0;

      for (int p = m; p < 10000; p++) {

        double probabilities = dist(generator);

        if (probabilities > max_weight) {
          max_weight = probabilities;
          m_star = p; 
        }
      }

      std::swap(particles.targets[t][m], particles.targets[t][m_star]);
    }
  }

  std::cout << "PROCESS Serial - took " << watch.ElapsedMs() << " ms" << std::endl;
  

  watch.Restart();
  __gnu_parallel::for_each(particles.targets.begin(), particles.targets.end(), [&](auto& target_subparticles) {
  // for (int t = 0; t < particles.targets.size(); ++t) {

    double max_weight;
    int m_star;

    for (int m = 0; m < 10000; ++m) {

      max_weight = -1.0;
      // m_star = -1;
      #pragma omp parallel
      {
        int m_star_calc = -1;
        double max_weight_calc = -1.0;
        double probabilities;

        #pragma omp for
        for (int p = m; p < 10000; ++p) {

          probabilities = dist(generator);

          if (probabilities > max_weight_calc) {
            max_weight_calc = probabilities;
            m_star_calc = p;
          }
        }

        #pragma omp critical
        {
          if (max_weight_calc > max_weight) {
            max_weight = max_weight_calc;
            m_star = m_star_calc;
          }
        }
      }

      std::swap(target_subparticles[m], target_subparticles[m_star]);
    }
  }, __gnu_parallel::parallel_unbalanced);

  std::cout << "ALGORITHM Parallel (OMP) - took " << watch.ElapsedMs() << " ms" << std::endl;
}

int main()
{
  warmUp();
  timeNormalizeWeights();
  timeResize();
  timeAlgorithmTarget();

  return EXIT_SUCCESS;
}
