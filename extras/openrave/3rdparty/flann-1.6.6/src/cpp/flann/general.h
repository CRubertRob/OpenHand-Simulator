/***********************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright 2008-2009  Marius Muja (mariusm@cs.ubc.ca). All rights reserved.
 * Copyright 2008-2009  David G. Lowe (lowe@cs.ubc.ca). All rights reserved.
 *
 * THE BSD LICENSE
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *************************************************************************/

#ifndef CONSTANTS_H
#define CONSTANTS_H

#define ARRAY_LEN(a) (sizeof(a)/sizeof(a[0]))

#ifdef _MSC_VER
#pragma warning(disable:4738) // storing 32-bit float result in memory, possible loss of performance
#pragma warning(disable:4365) // conversion from 'int' to 'size_t', signed/unsigned mismatch
#pragma warning(disable:4244) // conversion from 'const int' to 'float', possible loss of data
#pragma warning(disable:4061) // enumerator 'X' in switch of enum 'Y' is not explicitly handled by a case label
#pragma warning(disable:4710) // function not inlined
#pragma warning(disable:4711) // function selected for automatic inline expansion
#pragma warning(disable:4701) // potentially uninitialized variable
#pragma warning(disable:4305) // truncation from double to float
#pragma warning(disable:4820) // X  bytes padded after data member
#pragma warning(disable:4100) // unreferenced formal parameter
#pragma warning(disable:4245) // conversion from int to unsigned char, signed/unsigned mismatch
#endif

/* Nearest neighbour index algorithms */
enum flann_algorithm_t {
	LINEAR = 0,
	KDTREE = 1,
	KMEANS = 2,
	COMPOSITE = 3,
	KDTREE_SINGLE = 4,
	SAVED = 254,
	AUTOTUNED = 255
};

enum flann_centers_init_t {
	CENTERS_RANDOM = 0,
	CENTERS_GONZALES = 1,
	CENTERS_KMEANSPP = 2
};

enum flann_log_level_t {
	LOG_NONE = 0,
	LOG_FATAL = 1,
	LOG_ERROR = 2,
	LOG_WARN = 3,
	LOG_INFO = 4
};

enum flann_distance_t {
	EUCLIDEAN = 1,
	MANHATTAN = 2,
	MINKOWSKI = 3,
	MAX_DIST   = 4,
	HIST_INTERSECT   = 5,
	HELLINGER = 6,
	CS        = 7,
	CHI_SQUARE = 7,
	KL        = 8,
	KULLBACK_LEIBLER        = 8
};

enum flann_datatype_t {
	FLANN_INT8 = 0,
	FLANN_INT16 = 1,
	FLANN_INT32 = 2,
	FLANN_INT64 = 3,
	FLANN_UINT8 = 4,
	FLANN_UINT16 = 5,
	FLANN_UINT32 = 6,
	FLANN_UINT64 = 7,
	FLANN_FLOAT32 = 8,
	FLANN_FLOAT64 = 9
};

const int CHECKS_UNLIMITED = -1;
const int CHECKS_AUTOTUNED = -2;


struct FLANNParameters {
	enum flann_algorithm_t algorithm; /* the algorithm to use */

	/* search time parameters */
	int checks;                /* how many leafs (features) to check in one search */
    float cb_index;            /* cluster boundary index. Used when searching the kmeans tree */
    float eps;					/* eps parameter for eps-knn search */

    /*  kdtree index parameters */
    int trees;                 /* number of randomized trees to use (for kdtree) */
    int leaf_max_size;

    /* kmeans index parameters */
	int branching;             /* branching factor (for kmeans tree) */
	int iterations;            /* max iterations to perform in one kmeans cluetering (kmeans tree) */
	enum flann_centers_init_t centers_init;  /* algorithm used for picking the initial cluster centers for kmeans tree */

	/* autotuned index parameters */
	float target_precision;    /* precision desired (used for autotuning, -1 otherwise) */
	float build_weight;        /* build tree time weighting factor */
	float memory_weight;       /* index memory weigthing factor */
    float sample_fraction;     /* what fraction of the dataset to use for autotuning */

    /* other parameters */
    enum flann_log_level_t log_level;    /* determines the verbosity of each flann function */
	long random_seed;          		/* random seed to use */
};



#ifdef __cplusplus

#include <stdexcept>
#include <cassert>
#include "flann/util/object_factory.h"

namespace flann {

class FLANNException : public std::runtime_error {
 public:
   FLANNException(const char* message) : std::runtime_error(message) { }

   FLANNException(const std::string& message) : std::runtime_error(message) { }
 };


struct IndexParams {
protected:
	IndexParams(flann_algorithm_t algorithm_) : algorithm(algorithm_) {};

public:
	static IndexParams* createFromParameters(const FLANNParameters& p);

	virtual flann_algorithm_t getIndexType() const { return algorithm; };

	virtual void fromParameters(const FLANNParameters& p) = 0;
	virtual void toParameters(FLANNParameters& p) const = 0;

	virtual void print() const = 0;

	flann_algorithm_t algorithm;
};


typedef ObjectFactory<IndexParams, flann_algorithm_t> ParamsFactory;


struct SearchParams {
	SearchParams(int checks_ = 32, float eps_ = 0, bool sorted_ = true ) :
		checks(checks_), eps(eps_), sorted(sorted_) {};

	int checks;		// how many leafs to visit when searching for neighbours (-1 for unlimited)
	float eps;		// search for eps-approximate neighbours (default: 0)
	bool sorted;	// only for radius search, require neighbours sorted by distance (default: true)
};

}

#endif

#endif  /* CONSTANTS_H */
