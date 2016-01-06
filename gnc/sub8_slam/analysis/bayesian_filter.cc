// sudo apt-get install mathgl libmgl-dev
#include <vector>
#include <ctime>
#include <boost/math/distributions/uniform.hpp>
#include <boost/math/distributions/normal.hpp>
#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>
#include <mgl2/qt.h>
#include <iostream>

/*
TODO:
  - Do an alpha-beta parametric description
    - Maybe: Use ArrayFire/CL to do explicit computation of p_xn instead of lossy parametric

*/

double max_Z = 10.0;
std::vector<double> my_observations = {0.1, 0.9, 0.6, 0.2};

double p_xn(double x, double Z, double pi, double tau2=0.005);

void mgls_prepare2d(mglData *a, std::vector<double>& observations, mglData *v = 0) {
  register long i, j, n = 200, m = 200, i0;
  if (a)
    a->Create(n, m);
  if (v) {
    v->Create(9);
    v->Fill(-1, 1);
  }

  mreal Z, pi;
  double max_likelihood = 0.;
  for (i = 0; i < n; i++) {
    Z = (i / (n - 1.)) * max_Z;
    for (j = 0; j < m; j++) {
      pi = j / (m - 1.);
      i0 = i + n * j;

      double likelihood = 1.0;

      for (unsigned int obs_num = 0; obs_num < observations.size(); obs_num++){
        likelihood *= p_xn(observations[obs_num], Z, pi);
      }

      if (likelihood > max_likelihood) {
        max_likelihood = likelihood;
      }

      if (a) {
        a->a[i0] = static_cast<mreal> (log(likelihood));
      }
    }
  }
}

double p_xn(double x, double Z, double pi, double tau2) {
  /* The probability is exactly a scalar
    - N(xn | Z, tau**2) --> N~(Z, tau**2) evaluated at xn
  --> Hope the compiler inlines
  */
  double p;
  // TODO: Avoid reevaluating (We can reuse normal(x) for every pi)
  boost::math::normal_distribution<double> normal_x(Z, tau2);
  boost::math::uniform_distribution<double> uniform_outlier(0.0, max_Z);
  p = (pi * boost::math::pdf(normal_x, x)) +
      ((1.0 - pi) * boost::math::pdf(uniform_outlier, x));

  return p;

}

int sample(mglGraph *gr) {
  mglData a;

  mgls_prepare2d(&a, my_observations);

  // gr->SubPlot(2, 2, 0, "");
  gr->Title("Likelihood for depth and inlier fraction");
  gr->Box();
  gr->Dens(a);
  gr->SetRanges(0.0, 1, 0.0, 1);
  gr->Axis();
  gr->Label('x', "Depth");
  gr->Label('y', "\\Inlier Probability");
  return 0;
}

int main() {
  mglGraph* gr = new mglGraph();
  boost::random::mt19937 rng(std::time(0));
  boost::normal_distribution<double> nd(0.3, 0.01);
  boost::variate_generator<boost::mt19937&,
                           boost::normal_distribution<double> > var_nor(rng, nd);

  boost::random::uniform_real_distribution<double> selector(0.0, 1.0);

  double true_pi = 0.3;

  double decision;
  double new_observation;

  for(int k = 0; k < 0; k++) {
    decision = selector(rng);
    if (decision > true_pi) {
      new_observation = selector(rng);
    } else {
      new_observation = var_nor();
    }
    my_observations.push_back(new_observation);
    std::cout << "New observation " << new_observation << std::endl;
  }
  // [1] http://mathgl.sourceforge.net/doc_en/Animation.html
  gr->StartGIF("sample.gif");

  for(int k = 0; k < 60; k++) {
    decision = selector(rng);
    if (decision > true_pi) {
      new_observation = selector(rng);
    } else {
      new_observation = var_nor();
    }
    my_observations.push_back(new_observation);
    std::cout << "New observation " << new_observation << std::endl;

    if (k % 2) {
      continue;
    }

    gr->NewFrame();
    sample(gr);
    gr->EndFrame();
    // gr->WriteFrame();
    // mglQT gr(sample, "MathGL examples");


    // gr.Run();
  }
  gr->CloseGIF();
  return 0;

  // gr.WriteFrame("test.png");
}
