/*
 * myRobot.cc
 *
 *  created on: 21.01.2016
 *      author: rungger
 */

/*
 * information about this example is given in the readme file
 *
 */

#include <array>
#include <iostream>

#include "cuddObj.hh"

#include "SymbolicSet.hh"
#include "SymbolicModelGrowthBound.hh"

#include "TicToc.hh"
#include "RungeKutta4.hh"
#include "FixedPoint.hh"


/* state space dim */
#define sDIM 3
#define iDIM 2

/* data types for the ode solver */
typedef std::array<double,3> state_type;
typedef std::array<double,2> input_type;

/* sampling time */
const double tau = 0.7;
/* number of intermediate steps in the ode solver */
const int nint=5;
OdeSolver ode_solver(sDIM,nint,tau);

/* we integrate the unicycle ode by 0.3 sec (the result is stored in x)  */
auto  unicycle_post = [](state_type &x, input_type &u) -> void {


 //the ode describing the bot 
    auto rhs =[](state_type& xx,  const state_type &x, input_type &u) -> void {
        xx[0] = (u[0]+u[1])*0.5*std::cos(x[2])*0.7459991;
        xx[1] = (u[0]+u[1])*0.5*std::sin(x[2])*0.7459991;
        xx[2] = (u[1]-u[0])/105.4*0.678181;
    };


/*
 // the ode describing the unicycle 
  auto rhs =[](state_type& xx,  const state_type &x, input_type &u) -> void {
      xx[0] = u[0]*std::cos(x[2]);
      xx[1] = u[0]*std::sin(x[2]);
      xx[2] = u[1];
  };
*/
  ode_solver(rhs,x,u);
};

/* computation of the growth bound (the result is stored in r)  */
auto radius_post = [](state_type &r, const input_type &u) -> void{

    r[0] = r[0]+r[2]*std::abs((u[1]-u[0])/105.4*0.678181)*0.7;
    r[1] = r[1]+r[2]*std::abs((u[1]-u[0])/105.4*0.678181)*0.7;
};


/* forward declaration of the functions to setup the state space 
 * and input space of the unicycle example */
scots::SymbolicSet unicycleCreateStateSpace(Cudd &mgr);
scots::SymbolicSet unicycleCreateInputSpace(Cudd &mgr);


int main() {
  /* to measure time */
  TicToc tt;
  /* there is one unique manager to organize the bdd variables */
  Cudd mgr;

  /****************************************************************************/
  /* construct SymbolicSet for the state space */
  /****************************************************************************/
  scots::SymbolicSet ss=unicycleCreateStateSpace(mgr);
  ss.writeToFile("myRobot_ss.bdd");
  /* write SymbolicSet of obstacles to unicycle_obst.bdd */
  ss.complement();
  ss.writeToFile("myRobot_obst.bdd");
  ss.complement();
  std::cout << "Unfiorm grid details:" << std::endl;
  ss.printInfo(1);

  /****************************************************************************/
  /* the target set */
  /****************************************************************************/
  /* first make a copy of the state space so that we obtain the grid
   * information in the new symbolic set */
  scots::SymbolicSet ts = ss;
  /* define the target set as a symbolic set */
    double H[4*sDIM]={-1, 0, 0,
                    1, 0, 0,
                    0, -1, 0,
                    0, 1, 0};
  /* compute inner approximation of P={ x | H x<= h1 }  */
  double c[4] = {-450,590,-600,840};
  ts.addPolytope(4,H,c, scots::INNER);
  ts.writeToFile("myRobot_ts.bdd");


  /****************************************************************************/
  /* construct SymbolicSet for the input space */
  /****************************************************************************/
  scots::SymbolicSet is=unicycleCreateInputSpace(mgr);
  std::cout << std::endl << "Input space details:" << std::endl;
  is.printInfo(1);

  /****************************************************************************/
  /* setup class for symbolic model computation */
  /****************************************************************************/
  /* first create SymbolicSet of post variables 
   * by copying the SymbolicSet of the state space and assigning new BDD IDs */
  scots::SymbolicSet sspost(ss,1);
  /* instantiate the SymbolicModel */
  scots::SymbolicModelGrowthBound<state_type,input_type> abstraction(&ss, &is, &sspost);
  /* compute the transition relation */
  tt.tic();
  abstraction.computeTransitionRelation(unicycle_post, radius_post);
  std::cout << std::endl;
  tt.toc();
  /* get the number of elements in the transition relation */
  std::cout << std::endl << "Number of elements in the transition relation: " << abstraction.getSize() << std::endl;


  /************************************ ****************************************/
  /* we continue with the controller synthesis */
  /****************************************************************************/
  int verbose=1;
  /* we setup a fixed point object to compute reachabilty controller */
  scots::FixedPoint fp(&abstraction);
  /* the fixed point algorithm operates on the BDD directly */
  BDD T = ts.getSymbolicSet();
  tt.tic();
  BDD C = fp.reach(T,verbose);
  tt.toc();

  /****************************************************************************/
  /* last we store the controller as a SymbolicSet 
   * the underlying uniform grid is given by the Cartesian product of 
   * the uniform gird of the space and uniform gird of the input space */
  /****************************************************************************/
  scots::SymbolicSet controller(ss,is);
  controller.setSymbolicSet(C);
  controller.writeToFile("myRobot_controller.bdd");

  scots::SymbolicSet tr = abstraction.getTransitionRelation();
  tr.writeToFile("myRobot_transition_relation.bdd");


  
  return 1;
}

scots::SymbolicSet unicycleCreateStateSpace(Cudd &mgr) {

  /* setup the workspace of the synthesis problem and the uniform grid */
  /* lower bounds of the hyper rectangle */
  double lb[sDIM]={425,580,-M_PI-0.4};  
  /* upper bounds of the hyper rectangle */
  double ub[sDIM]={1980,1900,M_PI+0.4}; 
  /* grid node distance diameter */
  double eta[sDIM]={30,30,.2};   



  /* eta is added to the bound so as to ensure that the whole
   * [0,10]x[0,10]x[-pi-eta,pi+eta] is covered by the cells */

  scots::SymbolicSet ss(mgr,sDIM,lb,ub,eta);

  /* add the grid points to the SymbolicSet ss */
  ss.addGridPoints();
  

  /* remove the obstacles from the state space */
  /* the obstacles are defined as polytopes */
  /* define H* x <= h */
  double H[4*sDIM]={-1, 0, 0,
                    1, 0, 0,
                    0, -1, 0,
                    0, 1, 0};

  
  /****************************************************************************/
  /* set up constraint functions with obtacles */
  /****************************************************************************/
  //double c1= 0.1/2.0+1e-16;
  //double c2= 0.1/2.0+1e-16;
  double h[4] = {-622,1800,-790,1700};

  /* remove outer approximation of P={ x | H x<= h1 } form state space */
   ss.remPolytope(4,H,h, scots::OUTER);
 return ss;
}

scots::SymbolicSet unicycleCreateInputSpace(Cudd &mgr) {

  /* lower bounds of the hyper rectangle */
  double lb[sDIM]={-240,-240};  
  /* upper bounds of the hyper rectangle */
  double ub[sDIM]={240,240}; 
  /* grid node distance diameter */
  double eta[sDIM]={120,120};   

  scots::SymbolicSet is(mgr,iDIM,lb,ub,eta);
  is.addGridPoints();

  return is;
}

