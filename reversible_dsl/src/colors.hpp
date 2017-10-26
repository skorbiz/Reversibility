#ifndef COLORS_HPP
#define COLORS_HPP

#include <iostream>

#define CRED( var ) std::cout << "\033[31m" << (var) << "\033[0m" << std::endl

namespace dsl {
namespace color {

extern const std::string COLORRESET;	/* Reset color */
extern const std::string DEFAULT;		/* Reset color */
extern const std::string BLACK;			/* Black */
extern const std::string RED;			/* Red */
extern const std::string GREEN;   		/* Green */
extern const std::string YELLOW;  		/* Yellow */
extern const std::string BLUE;    		/* Blue */
extern const std::string MAGENTA; 		/* Magenta */
extern const std::string CYAN;    		/* Cyan */
extern const std::string WHITE;   		/* White */
extern const std::string BOLDBLACK;   	/* Bold Black */
extern const std::string BOLDRED;     	/* Bold Red */
extern const std::string BOLDGREEN;   	/* Bold Green */
extern const std::string BOLDYELLOW;  	/* Bold Yellow */
extern const std::string BOLDBLUE;    	/* Bold Blue */
extern const std::string BOLDMAGENTA; 	/* Bold Magenta */
extern const std::string BOLDCYAN;    	/* Bold Cyan */
extern const std::string BOLDWHITE;		/* Bold White */

std::string red(std::string input);


//const std::string CLEAR = "\033[2J"  // clear screen escape code

} /* namespace color */
} /* namespace dsl*/

#endif // COLORS_HPP



//void colorfunc(double v)
//{
//	if(v > 0.9)
//		std::cout << dsl::color::BOLDRED;
//	else if(v > 0.8)
//		std::cout << dsl::color::BOLDMAGENTA ;
//	else if(v > 0.7)
//		std::cout << dsl::color::BOLDYELLOW ;
//	else if(v > 0.6)
//		std::cout << dsl::color::BOLDBLUE ;
//	else if(v > 0.5)
//		std::cout << dsl::color::BOLDCYAN ;
//	else if(v > 0.4)
//		std::cout << dsl::color::RED ;
//	else if(v > 0.3)
//		std::cout << dsl::color::MAGENTA ;
//	else if(v > 0.2)
//		std::cout << dsl::color::YELLOW ;
//	else if(v > 0.1)
//		std::cout << dsl::color::BLUE ;
//	else if(v > 0.05)
//		std::cout << dsl::color::CYAN ;
//	else
//		std::cout << dsl::color::WHITE ;
//}
//
//void colorfunc2(double v)
//{
//	if(v > 2.9)
//		std::cout << dsl::color::BOLDRED ;
//	else if(v > 2.8)
//		std::cout << dsl::color::BOLDMAGENTA ;
//	else if(v > 2.7)
//		std::cout << dsl::color::BOLDYELLOW ;
//	else if(v > 2.6)
//		std::cout << dsl::color::BOLDBLUE ;
//	else if(v > 2.5)
//		std::cout << dsl::color::BOLDCYAN ;
//	else if(v > 2.4)
//		std::cout << dsl::color::RED ;
//	else if(v > 2.3)
//		std::cout << dsl::color::MAGENTA ;
//	else if(v > 2.2)
//		std::cout << dsl::color::YELLOW ;
//	else if(v > 2.1)
//		std::cout << dsl::color::BLUE ;
//	else if(v > 2.0)
//		std::cout << dsl::color::CYAN ;
//	else
//		std::cout << dsl::color::WHITE ;
//}
