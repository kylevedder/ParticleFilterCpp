#pragma once

static constexpr bool kProduction = false;
static constexpr float kPi = M_PI;
static constexpr float kEpsilon = 0.001f;

#define NP_CHECK(exp) \
  if (!kProduction) { \
    CHECK(exp);       \
  }

#define NP_CHECK_MSG(exp, message) \
  if (!kProduction) {              \
    CHECK(exp) << ": " << message; \
  }

#define NP_NOT_NAN(exp)      \
  if (!kProduction) {        \
    CHECK(!std::isnan(exp)); \
  }

#define NP_NOT_NULL(exp) \
  if (!kProduction) {    \
    CHECK_NOTNULL(exp);  \
  }

#define NP_CHECK_EQ(exp1, exp2) \
  if (!kProduction) {           \
    CHECK_EQ(exp1, exp2);       \
  }

#define NP_CHECK_EQ_MSG(exp1, exp2, message) \
  if (!kProduction) {                        \
    CHECK_EQ(exp1, exp2) << ": " << message; \
  }
