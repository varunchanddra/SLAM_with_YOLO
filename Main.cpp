/*
 * This file is part of ElasticFusion.
 *
 * Copyright (C) 2015 Imperial College London
 * 
 * The use of the code within this file and all code within files that 
 * make up the software that is ElasticFusion is permitted for 
 * non-commercial purposes only.  The full terms and conditions that 
 * apply to the code within this file are detailed within the LICENSE.txt 
 * file and at <http://www.imperial.ac.uk/dyson-robotics-lab/downloads/elastic-fusion/elastic-fusion-license/> 
 * unless explicitly stated.  By downloading this file you agree to 
 * comply with these terms.
 *
 * If you wish to use any of this code for commercial purposes then 
 * please email researchcontracts.engineering@imperial.ac.uk.
 *
 */

#include "MainController.h"

MainController * mainc;

extern "C" void addFrames(char * rgb, char * dep)
{
	mainc->addFrames(rgb, dep);
}

extern "C" void fun_start()
{
	char* args [3] = {"./a.out", "-l", "~/Downloads/dyson_lab.klg"};
  	
  	// arg?v[1] = "-l";
  	// argv?[2] = "~/Downloads/dyson_lab.klg";

    mainc = new MainController(3, (char **)args);

    mainc->launch();

    // return 0;
}
