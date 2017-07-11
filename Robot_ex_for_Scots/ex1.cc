/*
 * ex1.cc
 *
 *  created on: 31.11.2015
 *      author: m.khaled
 */

#include <vector>
#include <iostream>
#include "imhotep.hh"


#define BDD_FILE "scots-files-vehicle/myRobot_controller.bdd"

#define ssDIM 3
#define isDIM 2

#define DCONTR_FILE "vehicle_contr_determinized.bdd"
#define VHDL_FILE "bdd_myRobot.v"

int main() {
	Cudd mgr;

	BDDReader reader(mgr, BDD_FILE, BDD_FILE_TYPE::SCOTS_BDD);
	BDD contr = reader.ReadBdd();
	size_t nBddvars_states  = reader.getNumBddVars(0, ssDIM);
	size_t nBddvars_actions = reader.getNumBddVars(ssDIM, isDIM);

	cout << "nBddvars_states : " << nBddvars_states << endl;
	cout << "nBddvars_actions: " << nBddvars_actions << endl;

	BddDeterminizer determinizer(mgr, contr, nBddvars_states, nBddvars_actions);
	contr = determinizer.determinize(DETERMINIZE_METHOD::RANDOM, true, 1);

	cout << "Saving the determinized controller ... ";
	SCOTSBDDReader scots_reader = reader.getScotsReader();
	scots_reader.setBDD(contr);
	scots_reader.writeToFile(DCONTR_FILE);

	BddToHDL converter(mgr, contr, nBddvars_states, nBddvars_actions);
	converter.GenerateRawVerilog(VHDL_FILE, true, 1);

	cout << "done !";
	return 1;
}
