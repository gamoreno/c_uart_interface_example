/*
 * utils.cpp
 *
 *  Created on: Sep 13, 2017
 *      Author: gmoreno
 */

#include "utils.h"
#include <string.h>
#include <boost/tokenizer.hpp>

using namespace std;

std::vector<std::string> splitString(const std::string& str, const std::string& separators) {

	vector<string> tokens;
    typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
	boost::char_separator<char> sep(separators.c_str());
	tokenizer tok(str, sep);

	for (tokenizer::const_iterator it = tok.begin(); it != tok.end(); ++it) {
		tokens.push_back(*it);
	}

	return tokens;
}
