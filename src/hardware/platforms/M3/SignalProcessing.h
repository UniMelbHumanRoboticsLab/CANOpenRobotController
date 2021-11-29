/**
 *
 * \file SignalProcessing.h
 * \author Vincent Crocher
 * \version 0.1
 * \date 2021-05-06
 * \copyright Copyright (c) 2021
 *
 * \brief  Signal processing: A basic IIR filtering class and differentiator.
 *
 */
#ifndef SIGNALPROCESSING_H
#define SIGNALPROCESSING_H

#include <vector>
#include <iostream>
#include <Eigen/Dense>


class Filter
{
  public:
    /**
       * \brief Constructor allocating for a degree n filter. Still requires to be initiaised.
       *
       * \param order: order of the filter
       * \param init_element: Both used for the size of the signal to be filtered and initialisation values of y and x
       */
    Filter(unsigned int order, Eigen::VectorXd init_element);
    /**
       * \brief Constructor allocating for a degree n filter and initialise it with parameters a and b.
       *
       * \param order: order of the filter
       * \param init_element: Both used for the size of the signal to be filtered and initialisation values of y and x
       * \param b: vector of length order containing filter numerator coefficients
       * \param a: vector of length order containing filter denominator coefficients
       */
    Filter(unsigned int order, Eigen::VectorXd init_element, std::vector<double> b, std::vector<double> a);
    ~Filter();

    /**
       * \brief Initialise the filter with parameters a and b.
       *
       * \param b: vector of length order containing filter numerator coefficients
       * \param a: vector of length order containing filter denominator coefficients
       */
    void init(std::vector<double> b_, std::vector<double> a_);

    /**
       * \brief Initialise the filter as a Butterworth low-pass of order 2 and normalised cutoff frequency fn.
       *
       * \param fn: normalised cutoff frequency of the filter.
       */
    void initButter2low(double fn);

    /**
       * \brief Add new element and apply the filter (if initialised);
       *
       * \param elem: New signal element to be filtered
       * \return filtered signal
       */
    Eigen::VectorXd filt(Eigen::VectorXd elem);

    bool isInitialised() {return initialised;}

  protected:

  private:
    bool initialised = false;
    unsigned int order = 0;             //!< Order of the filter
    std::vector<double> a, b;           //!< Denominator and numerator coefficients of the filter
    std::vector<Eigen::VectorXd> y, x;  //!< Saved input and output of the signal, of length order+1
};

#endif // SIGNALPROCESSING_H
