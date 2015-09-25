//
// Created by Ryan Taber on 9/25/15.
//

#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE sample
#include <boost/test/unit_test.hpp>

int add(int x, int y){
    return x+y;
}

BOOST_AUTO_TEST_SUITE(test_suite1)

BOOST_AUTO_TEST_CASE(test_case1){
        BOOST_CHECK(add(2,2) == 4);
}

BOOST_AUTO_TEST_CASE(test_case2){
        BOOST_CHECK(2, 1+1);
}

BOOST_AUOT_TEST_SUITE_END()
