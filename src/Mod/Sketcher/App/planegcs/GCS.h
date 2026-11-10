/***************************************************************************
 *   Copyright (c) 2011 Konstantinos Poulios <logari81@gmail.com>          *
 *                                                                         *
 *   This file is part of the FreeCAD CAx development system.              *
 *                                                                         *
 *   This library is free software; you can redistribute it and/or         *
 *   modify it under the terms of the GNU Library General Public           *
 *   License as published by the Free Software Foundation; either          *
 *   version 2 of the License, or (at your option) any later version.      *
 *                                                                         *
 *   This library  is distributed in the hope that it will be useful,      *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU Library General Public License for more details.                  *
 *                                                                         *
 *   You should have received a copy of the GNU Library General Public     *
 *   License along with this library; see the file COPYING.LIB. If not,    *
 *   write to the Free Software Foundation, Inc., 59 Temple Place,         *
 *   Suite 330, Boston, MA  02111-1307, USA                                *
 *                                                                         *
 ***************************************************************************/

#ifndef PLANEGCS_GCS_H
#define PLANEGCS_GCS_H

#include <Eigen/QR>

#include "../../SketcherGlobal.h"
#include "SubSystem.h"


#define EIGEN_VERSION \
    (EIGEN_WORLD_VERSION * 10000 + EIGEN_MAJOR_VERSION * 100 + EIGEN_MINOR_VERSION)

#if EIGEN_VERSION >= 30202
# define EIGEN_SPARSEQR_COMPATIBLE
# include <Eigen/Sparse>
#endif

namespace GCS
{
///////////////////////////////////////
// Other BFGS Solver parameters
///////////////////////////////////////
#define XconvergenceRough 1e-8
#define smallF 1e-20

///////////////////////////////////////
// Solver
///////////////////////////////////////

enum SolveStatus
{
    Success = 0,                    // Found a solution zeroing the error function
    Converged = 1,                  // Found a solution minimizing the error function
    Failed = 2,                     // Failed to find any solution
    SuccessfulSolutionInvalid = 3,  // This is a solution where the solver succeeded, but the
                                    // resulting geometry is OCE-invalid
};

enum Algorithm
{
    BFGS = 0,
    LevenbergMarquardt = 1,
    DogLeg = 2
};

enum DogLegGaussStep
{
    FullPivLU = 0,
    LeastNormFullPivLU = 1,
    LeastNormLdlt = 2,
    // The DogLeg Gauss-Newton step from sparse matrices, with no J J^T or J^T J formed and no QR. The
    // step is h = J^T (J J^T + delta I)^-1 q refined as described at sparseGaussNewtonStep: for J of
    // full row rank it is the least-norm step (as LeastNormLdlt); for a consistent rank-deficient J
    // (redundant constraints that agree) it is the minimum-norm solution of J h = -f; for an
    // inconsistent J (conflicting constraints, or redundant nonlinear rows away from a solution) it
    // is the minimum-norm solution of the least-squares problem min |J h + f|, which is FullPivLU's
    // answer only when J has full column rank. Directions with a singular value below about
    // sqrt(delta) = 3e-5 |J_i|max are damped (Levenberg-like) instead of inverted. Only the DogLeg
    // solver reads it. Without Eigen's sparse module (before 3.2.2) it behaves as LeastNormLdlt.
    SparseLeastNorm = 3
};

enum QRAlgorithm
{
    EigenDenseQR = 0,
    EigenSparseQR = 1
};

enum DebugMode
{
    NoDebug = 0,
    Minimal = 1,
    IterationLevel = 2
};

// Magic numbers for Constraint tags
// - Positive Tags identify a higher level constraint form which the solver constraint
// originates
// - Negative Tags represent temporary constraints, used for example in moving operations, these
// have a different handling in component splitting, see GCS::initSolution. Lifetime is defined
// by the container object via GCS::clearByTag.
//      -   -1 is typically used as tag for these temporary constraints, its parameters are
//      enforced with
//          a lower priority than the main system (real sketcher constraints). It gives a nice
//          effect when dragging the edge of an unconstrained circle, that the center won't move
//          if the edge can be dragged, and only when/if the edge cannot be dragged, e.g. radius
//          constraint, the center is moved).
enum SpecialTag
{
    DefaultTemporaryConstraint = -1
};

class SketcherExport System
{
    // This is the main class. It holds all constraints and information
    // about partitioning into subsystems and solution strategies
private:
    VEC_pD plist;        // list of the unknown parameters
    VEC_pD pdrivenlist;  // list of parameters of driven constraints
    MAP_pD_I pIndex;

    VEC_pD pDependentParameters;  // list of dependent parameters by the system

    // This is a map of primary and secondary identifiers that are found dependent by the solver
    // GCS ignores from a type point
    std::vector<std::vector<double*>> pDependentParametersGroups;

    std::vector<Constraint*> clist;
    std::map<Constraint*, VEC_pD> c2p;                // constraint to parameter adjacency list
    std::map<double*, std::vector<Constraint*>> p2c;  // parameter to constraint adjacency list

    std::vector<SubSystem*> subSystems, subSystemsAux;
    void clearSubSystems();

    VEC_D reference;
    void setReference();      // copies the current parameter values to reference
    void resetToReference();  // reverts all parameter values to the stored reference

    std::vector<VEC_pD> plists;  // partitioned plist except equality constraints
    // partitioned clist except equality constraints
    std::vector<std::vector<Constraint*>> clists;
    std::vector<MAP_pD_pD> reductionmaps;  // for simplification of equality constraints

    int dofs;
    std::set<Constraint*> redundant;
    VEC_I conflictingTags, redundantTags, partiallyRedundantTags;

    bool hasUnknowns;   // if plist is filled with the unknown parameters
    bool hasDiagnosis;  // if dofs, conflictingTags, redundantTags are up to date
    bool isInit;        // if plists, clists, reductionmaps are up to date

    bool emptyDiagnoseMatrix;  // false only if there is at least one driving constraint.

    // true while the redundancy state was supplied through solveWithSuppliedRedundancy() and
    // diagnose() has not replaced it
    bool diagnosisSupplied;

    int solve_BFGS(SubSystem* subsys, bool isFine = true, bool isRedundantsolving = false);
    int solve_LM(SubSystem* subsys, bool isRedundantsolving = false);
    int solve_DL(SubSystem* subsys, bool isRedundantsolving = false);
    // solve_DL() with sparse Jacobians and the sparse Gauss-Newton step (SparseLeastNorm)
    int solve_DLSparse(SubSystem* subsys, bool isRedundantsolving = false);

    void makeReducedJacobian(
        Eigen::MatrixXd& J,
        std::map<int, int>& jacobianconstraintmap,
        GCS::VEC_pD& pdiagnoselist,
        std::map<int, int>& tagmultiplicity
    );

    // sparseDiagnosis: the reduced Jacobian holds only its structural nonzeros (rows are the
    // driving constraints, with no zero padding for the other constraints).
    void makeReducedJacobianSparse(
        Eigen::SparseMatrix<double>& J,
        std::map<int, int>& jacobianconstraintmap,
        GCS::VEC_pD& pdiagnoselist,
        std::map<int, int>& tagmultiplicity
    );

    // diagnose() for sparseDiagnosis: sparse Jacobian, sparse QR and no dense R matrix.
    int diagnoseSparse(Algorithm alg);

    void makeDenseQRDecomposition(
        const Eigen::MatrixXd& J,
        const std::map<int, int>& jacobianconstraintmap,
        Eigen::FullPivHouseholderQR<Eigen::MatrixXd>& qrJT,
        int& rank,
        Eigen::MatrixXd& R,
        bool transposeJ = true,
        bool silent = false
    );

#ifdef EIGEN_SPARSEQR_COMPATIBLE
    void makeSparseQRDecomposition(
        const Eigen::MatrixXd& J,
        const std::map<int, int>& jacobianconstraintmap,
        Eigen::SparseQR<Eigen::SparseMatrix<double>, Eigen::COLAMDOrdering<int>>& SqrJT,
        int& rank,
        Eigen::MatrixXd& R,
        bool transposeJ = true,
        bool silent = false
    );
#endif
#ifdef EIGEN_SPARSEQR_COMPATIBLE
    // Sparse QR of J (transposeJ false) or of its transpose, for sparseDiagnosis. The rank is set
    // as in the dense-Jacobian overload above.
    void makeSparseQRDecomposition(
        const Eigen::SparseMatrix<double>& J,
        Eigen::SparseQR<Eigen::SparseMatrix<double>, Eigen::COLAMDOrdering<int>>& SqrJ,
        int& rank,
        bool transposeJ
    );
#endif
    // This function name is long for a reason:
    // - Only for DenseQR
    // - Only for Transposed Jacobian QR decomposition
    void identifyDependentGeometryParametersInTransposedJacobianDenseQRDecomposition(
        const Eigen::FullPivHouseholderQR<Eigen::MatrixXd>& qrJT,
        const GCS::VEC_pD& pdiagnoselist,
        int paramsNum,
        int rank
    );

    template<typename T>
    void identifyConflictingRedundantConstraints(
        Algorithm alg,
        const T& qrJT,
        const std::map<int, int>& jacobianconstraintmap,
        const std::map<int, int>& tagmultiplicity,
        GCS::VEC_pD& pdiagnoselist,
        Eigen::MatrixXd& R,
        int constrNum,
        int rank,
        int& nonredundantconstrNum
    );

    // The part of identifyConflictingRedundantConstraints that follows the grouping of the
    // dependent constraints: it chooses the constraints to skip, confirms the choice by a solve and
    // fills redundant, conflictingTags, redundantTags and partiallyRedundantTags.
    // Solves the system of the non-skipped driving constraints for the confirmation step of the
    // diagnosis. The SubSystems it creates are returned in subsystems, and solved[i] is 1 when
    // subsystems[i] reached Success; the caller applies solutions and deletes the SubSystems. With
    // componentwiseDiagnosisSolve each connected component of the constraint-parameter graph is
    // solved on its own, otherwise one SubSystem holds all. The return value is the worst status.
    int solveRedundancyConfirmation(
        std::vector<Constraint*>& clistTmp,
        GCS::VEC_pD& params,
        Algorithm alg,
        std::vector<SubSystem*>& subsystems,
        std::vector<char>& solved
    );

    void identifyConflictingRedundantFromGroups(
        Algorithm alg,
        std::vector<std::vector<Constraint*>>& conflictGroups,
        const std::map<int, int>& tagmultiplicity,
        GCS::VEC_pD& pdiagnoselist,
        int constrNum,
        int& nonredundantconstrNum
    );

    void eliminateNonZerosOverPivotInUpperTriangularMatrix(Eigen::MatrixXd& R, int rank);

#ifdef EIGEN_SPARSEQR_COMPATIBLE
    void identifyDependentParametersSparseQR(
        const Eigen::MatrixXd& J,
        const std::map<int, int>& jacobianconstraintmap,
        const GCS::VEC_pD& pdiagnoselist,
        bool silent = true
    );
#endif

    void identifyDependentParametersDenseQR(
        const Eigen::MatrixXd& J,
        const std::map<int, int>& jacobianconstraintmap,
        const GCS::VEC_pD& pdiagnoselist,
        bool silent = true
    );

    template<typename T>
    void identifyDependentParameters(
        T& qrJ,
        Eigen::MatrixXd& Rparams,
        int rank,
        const GCS::VEC_pD& pdiagnoselist,
        bool silent = true
    );

#ifdef _GCS_EXTRACT_SOLVER_SUBSYSTEM_
    void extractSubsystem(SubSystem* subsys, bool isRedundantsolving);
#endif
public:
    int maxIter;
    int maxIterRedundant;
    bool sketchSizeMultiplier;  // if true note that the total number of iterations allowed is
                                // MaxIterations *xLength
    bool sketchSizeMultiplierRedundant;
    double convergence;
    double convergenceRedundant;
    QRAlgorithm qrAlgorithm;
    bool autoChooseAlgorithm;
    int autoQRThreshold;
    // If true, diagnose() assembles the reduced Jacobian as a sparse matrix and uses the sparse QR
    // without a dense R matrix, so no dense matrix proportional to the whole system is formed.
    // Upstream's rule is kept: with autoChooseAlgorithm true, a system with fewer than
    // autoQRThreshold parameters still takes the dense QR path (FreeCAD issue 10903, sparse QR
    // failing rank detection on aligned slots), so sparseDiagnosis changes only larger systems.
    // With autoChooseAlgorithm false the sparse path is used at every size. The dependent groups
    // are the same as those of the sparse QR path with a dense Jacobian. The QR of J^T has the
    // fill of the Cholesky factor of J J^T: constraints that share an unknown make it denser.
    // Without Eigen's sparse module (before 3.2.2) the switch has no effect.
    // Default false: diagnose() is as upstream.
    bool sparseDiagnosis;
    // If true, the solve that confirms the redundant constraints chosen by diagnose() solves each
    // connected component of the constraint-parameter graph with its own SubSystem, trust region
    // and convergence test, as solve() does for the whole sketch, instead of one SubSystem of all
    // constraints and parameters. Upstream applies nothing, and confirms no redundant constraint,
    // unless the one solve succeeds, so a single component that does not converge leaves every
    // other component unconfirmed. With this switch the solutions of the components that reach
    // Success are applied and their skipped constraints are confirmed; the skipped constraints of a
    // component that does not converge are judged at the values from before the solve.
    // Differences from the single solve: each component has its own iteration budget when
    // sketchSizeMultiplierRedundant is set; the tolerance of a constraint that depends on no
    // unknown is DL_tolfRedundant, which is the DogLeg tolerance (the other algorithms use their
    // own); and a system with no unknown at all is solved as upstream (Success). Default false:
    // as upstream.
    bool componentwiseDiagnosisSolve;
    // Statistics of DogLegGaussStep::SparseLeastNorm, never reset by the solver: the number of
    // Gauss-Newton steps taken, and the number of those whose refinement used all twelve passes without
    // the correction reaching 1e-14 of h (inconsistent or nearly singular systems).
    unsigned long sparseStepCalls;
    unsigned long sparseStepPassLimit;
    DogLegGaussStep dogLegGaussStep;
    double qrpivotThreshold;
    DebugMode debugMode;
    double LM_eps;
    double LM_eps1;
    double LM_tau;
    double DL_tolg;
    double DL_tolx;
    double DL_tolf;
    double LM_epsRedundant;
    double LM_eps1Redundant;
    double LM_tauRedundant;
    double DL_tolgRedundant;
    double DL_tolxRedundant;
    double DL_tolfRedundant;

public:
    System();
    /*System(std::vector<Constraint *> clist_);*/
    ~System();

    void clear();
    void clearByTag(int tagId);

    int addConstraint(Constraint* constr);
    void removeConstraint(Constraint* constr);

    // basic constraints
    int addConstraintEqual(
        double* param1,
        double* param2,
        int tagId = 0,
        bool driving = true,
        Constraint::Alignment internalalignment = Constraint::Alignment::NoInternalAlignment
    );
    int addConstraintProportional(
        double* param1,
        double* param2,
        double ratio,
        int tagId,
        bool driving = true
    );
    int addConstraintDifference(
        double* param1,
        double* param2,
        double* difference,
        int tagId = 0,
        bool driving = true
    );
    int addConstraintP2PDistance(Point& p1, Point& p2, double* distance, int tagId = 0, bool driving = true);
    int addConstraintP2PAngle(
        Point& p1,
        Point& p2,
        double* angle,
        double incrAngle,
        int tagId = 0,
        bool driving = true
    );
    int addConstraintP2PAngle(Point& p1, Point& p2, double* angle, int tagId = 0, bool driving = true);
    int addConstraintP2LDistance(Point& p, Line& l, double* distance, int tagId = 0, bool driving = true);
    int addConstraintPointOnLine(Point& p, Line& l, int tagId = 0, bool driving = true);
    int addConstraintPointOnLine(Point& p, Point& lp1, Point& lp2, int tagId = 0, bool driving = true);
    int addConstraintPointOnPerpBisector(Point& p, Line& l, int tagId = 0, bool driving = true);
    int addConstraintPointOnPerpBisector(
        Point& p,
        Point& lp1,
        Point& lp2,
        int tagId = 0,
        bool driving = true
    );
    int addConstraintParallel(Line& l1, Line& l2, int tagId = 0, bool driving = true);
    int addConstraintPerpendicular(Line& l1, Line& l2, int tagId = 0, bool driving = true);
    int addConstraintPerpendicular(
        Point& l1p1,
        Point& l1p2,
        Point& l2p1,
        Point& l2p2,
        int tagId = 0,
        bool driving = true
    );
    int addConstraintL2LAngle(Line& l1, Line& l2, double* angle, int tagId = 0, bool driving = true);
    int addConstraintL2LAngle(
        Point& l1p1,
        Point& l1p2,
        Point& l2p1,
        Point& l2p2,
        double* angle,
        int tagId = 0,
        bool driving = true
    );
    int addConstraintAngleViaPoint(
        Curve& crv1,
        Curve& crv2,
        Point& p,
        double* angle,
        int tagId = 0,
        bool driving = true
    );
    int addConstraintAngleViaTwoPoints(
        Curve& crv1,
        Curve& crv2,
        Point& p1,
        Point& p2,
        double* angle,
        int tagId = 0,
        bool driving = true
    );
    int addConstraintAngleViaPointAndParam(
        Curve& crv1,
        Curve& crv2,
        Point& p,
        double* cparam,
        double* angle,
        int tagId = 0,
        bool driving = true
    );
    int addConstraintAngleViaPointAndTwoParams(
        Curve& crv1,
        Curve& crv2,
        Point& p,
        double* cparam1,
        double* cparam2,
        double* angle,
        int tagId = 0,
        bool driving = true
    );
    int addConstraintMidpointOnLine(Line& l1, Line& l2, int tagId = 0, bool driving = true);
    int addConstraintMidpointOnLine(
        Point& l1p1,
        Point& l1p2,
        Point& l2p1,
        Point& l2p2,
        int tagId = 0,
        bool driving = true
    );
    int addConstraintTangentCircumf(
        Point& p1,
        Point& p2,
        double* rd1,
        double* rd2,
        bool internal = false,
        int tagId = 0,
        bool driving = true
    );
    int addConstraintTangentAtBSplineKnot(
        BSpline& b,
        Line& l,
        unsigned int knotindex,
        int tagId = 0,
        bool driving = true
    );

    // derived constraints
    int addConstraintP2PCoincident(Point& p1, Point& p2, int tagId = 0, bool driving = true);
    int addConstraintHorizontal(Line& l, int tagId = 0, bool driving = true);
    int addConstraintHorizontal(Point& p1, Point& p2, int tagId = 0, bool driving = true);
    int addConstraintVertical(Line& l, int tagId = 0, bool driving = true);
    int addConstraintVertical(Point& p1, Point& p2, int tagId = 0, bool driving = true);
    int addConstraintCoordinateX(Point& p, double* x, int tagId = 0, bool driving = true);
    int addConstraintCoordinateY(Point& p, double* y, int tagId = 0, bool driving = true);
    int addConstraintArcRules(Arc& a, int tagId = 0, bool driving = true);
    int addConstraintPointOnCircle(Point& p, Circle& c, int tagId = 0, bool driving = true);
    int addConstraintPointOnEllipse(Point& p, Ellipse& e, int tagId = 0, bool driving = true);
    int addConstraintPointOnHyperbolicArc(Point& p, ArcOfHyperbola& e, int tagId = 0, bool driving = true);
    int addConstraintPointOnParabolicArc(Point& p, ArcOfParabola& e, int tagId = 0, bool driving = true);
    int addConstraintPointOnBSpline(
        Point& p,
        BSpline& b,
        double* pointparam,
        int tagId,
        bool driving = true
    );
    int addConstraintArcOfEllipseRules(ArcOfEllipse& a, int tagId = 0, bool driving = true);
    int addConstraintCurveValue(Point& p, Curve& a, double* u, int tagId = 0, bool driving = true);
    int addConstraintArcOfHyperbolaRules(ArcOfHyperbola& a, int tagId = 0, bool driving = true);
    int addConstraintArcOfParabolaRules(ArcOfParabola& a, int tagId = 0, bool driving = true);
    int addConstraintPointOnArc(Point& p, Arc& a, int tagId = 0, bool driving = true);
    int addConstraintPerpendicularLine2Arc(Point& p1, Point& p2, Arc& a, int tagId = 0, bool driving = true);
    int addConstraintPerpendicularArc2Line(Arc& a, Point& p1, Point& p2, int tagId = 0, bool driving = true);
    int addConstraintPerpendicularCircle2Arc(
        Point& center,
        double* radius,
        Arc& a,
        int tagId = 0,
        bool driving = true
    );
    int addConstraintPerpendicularArc2Circle(
        Arc& a,
        Point& center,
        double* radius,
        int tagId = 0,
        bool driving = true
    );
    int addConstraintPerpendicularArc2Arc(
        Arc& a1,
        bool reverse1,
        Arc& a2,
        bool reverse2,
        int tagId = 0,
        bool driving = true
    );
    int addConstraintTangent(Line& l, Circle& c, int tagId = 0, bool driving = true);
    int addConstraintTangent(Line& l, Ellipse& e, int tagId = 0, bool driving = true);
    int addConstraintTangent(Line& l, Arc& a, int tagId = 0, bool driving = true);
    int addConstraintTangent(Circle& c1, Circle& c2, int tagId = 0, bool driving = true);
    int addConstraintTangent(Arc& a1, Arc& a2, int tagId = 0, bool driving = true);
    int addConstraintTangent(Circle& c, Arc& a, int tagId = 0, bool driving = true);

    int addConstraintCircleRadius(Circle& c, double* radius, int tagId = 0, bool driving = true);
    int addConstraintArcRadius(Arc& a, double* radius, int tagId = 0, bool driving = true);
    int addConstraintCircleDiameter(Circle& c, double* diameter, int tagId = 0, bool driving = true);
    int addConstraintArcDiameter(Arc& a, double* diameter, int tagId = 0, bool driving = true);
    int addConstraintEqualLength(Line& l1, Line& l2, int tagId = 0, bool driving = true);
    int addConstraintEqualRadius(Circle& c1, Circle& c2, int tagId = 0, bool driving = true);
    int addConstraintEqualRadii(Ellipse& e1, Ellipse& e2, int tagId = 0, bool driving = true);
    int addConstraintEqualRadii(ArcOfHyperbola& a1, ArcOfHyperbola& a2, int tagId = 0, bool driving = true);
    int addConstraintEqualRadius(Circle& c1, Arc& a2, int tagId = 0, bool driving = true);
    int addConstraintEqualRadius(Arc& a1, Arc& a2, int tagId = 0, bool driving = true);
    int addConstraintEqualFocus(ArcOfParabola& a1, ArcOfParabola& a2, int tagId = 0, bool driving = true);
    int addConstraintP2PSymmetric(Point& p1, Point& p2, Line& l, int tagId = 0, bool driving = true);
    int addConstraintP2PSymmetric(Point& p1, Point& p2, Point& p, int tagId = 0, bool driving = true);
    int addConstraintSnellsLaw(
        Curve& ray1,
        Curve& ray2,
        Curve& boundary,
        Point p,
        double* n1,
        double* n2,
        bool flipn1,
        bool flipn2,
        int tagId,
        bool driving = true
    );

    int addConstraintC2CDistance(Circle& c1, Circle& c2, double* dist, int tagId, bool driving = true);
    int addConstraintC2LDistance(Circle& c, Line& l, double* dist, int tagId, bool driving = true);
    int addConstraintP2CDistance(Point& p, Circle& c, double* distance, int tagId = 0, bool driving = true);
    int addConstraintArcLength(Arc& a, double* dist, int tagId, bool driving = true);

    // internal alignment constraints
    int addConstraintInternalAlignmentPoint2Ellipse(
        Ellipse& e,
        Point& p1,
        InternalAlignmentType alignmentType,
        int tagId = 0,
        bool driving = true
    );
    int addConstraintInternalAlignmentEllipseMajorDiameter(
        Ellipse& e,
        Point& p1,
        Point& p2,
        int tagId = 0,
        bool driving = true
    );
    int addConstraintInternalAlignmentEllipseMinorDiameter(
        Ellipse& e,
        Point& p1,
        Point& p2,
        int tagId = 0,
        bool driving = true
    );
    int addConstraintInternalAlignmentEllipseFocus1(
        Ellipse& e,
        Point& p1,
        int tagId = 0,
        bool driving = true
    );
    int addConstraintInternalAlignmentEllipseFocus2(
        Ellipse& e,
        Point& p1,
        int tagId = 0,
        bool driving = true
    );
    int addConstraintInternalAlignmentPoint2Hyperbola(
        Hyperbola& e,
        Point& p1,
        InternalAlignmentType alignmentType,
        int tagId = 0,
        bool driving = true
    );
    int addConstraintInternalAlignmentHyperbolaMajorDiameter(
        Hyperbola& e,
        Point& p1,
        Point& p2,
        int tagId = 0,
        bool driving = true
    );
    int addConstraintInternalAlignmentHyperbolaMinorDiameter(
        Hyperbola& e,
        Point& p1,
        Point& p2,
        int tagId = 0,
        bool driving = true
    );
    int addConstraintInternalAlignmentHyperbolaFocus(
        Hyperbola& e,
        Point& p1,
        int tagId = 0,
        bool driving = true
    );
    int addConstraintInternalAlignmentParabolaFocus(
        Parabola& e,
        Point& p1,
        int tagId = 0,
        bool driving = true
    );
    int addConstraintInternalAlignmentBSplineControlPoint(
        BSpline& b,
        Circle& c,
        unsigned int poleindex,
        int tag = 0,
        bool driving = true
    );
    int addConstraintInternalAlignmentKnotPoint(
        BSpline& b,
        Point& p,
        unsigned int knotindex,
        int tagId = 0,
        bool driving = true
    );

    double calculateAngleViaPoint(const Curve& crv1, const Curve& crv2, Point& p) const;
    double calculateAngleViaPoint(const Curve& crv1, const Curve& crv2, Point& p1, Point& p2) const;
    double calculateAngleViaParams(
        const Curve& crv1,
        const Curve& crv2,
        double* param1,
        double* param2
    ) const;
    void calculateNormalAtPoint(const Curve& crv, const Point& p, double& rtnX, double& rtnY) const;

    // Calculates errors of all constraints which have a tag equal to
    // the one supplied. Individual errors are summed up using RMS.
    // If none are found, NAN is returned
    // If there's only one, a signed value is returned.
    // Effectively, it calculates the error of a UI constraint
    double calculateConstraintErrorByTag(int tagId);

    void rescaleConstraint(int id, double coeff);

    void declareUnknowns(VEC_pD& params);
    void declareDrivenParams(VEC_pD& params);
    void initSolution(Algorithm alg = DogLeg);

    int solve(bool isFine = true, Algorithm alg = DogLeg, bool isRedundantsolving = false);
    int solve(VEC_pD& params, bool isFine = true, Algorithm alg = DogLeg, bool isRedundantsolving = false);
    // Solves with a redundancy set supplied by the caller instead of the internal diagnosis.
    // Driving constraints whose tag is in omittedTags (tag >= 0) are left out of the solved
    // system, as diagnosed redundant constraints are. diagnose() is not called, so no
    // whole-system Jacobian is formed. Partitioning, equality reduction and the per-component
    // solve are those of initSolution(); the solve is solve(isFine, alg, isRedundantsolving) with
    // one difference. Upstream solve() checks each omitted constraint at the parameter values from
    // before the solve, because the omitted constraint is not redirected to the SubSystem storage
    // that holds the solution until applySolution(). After this entry point the check is made at
    // the solved values: solve() writes the solution into the parameters, evaluates the omitted
    // constraints, and restores the previous values. A compatible omitted constraint therefore no
    // longer ends the solve with Converged.
    // The supplied set stands in for the diagnosis until diagnose() runs or a constraint with
    // tag >= 0 is added or removed: dofsNumber() is -1, the conflicting and partially redundant
    // lists are empty, and getRedundant() returns the omitted tags that name a driving constraint.
    // solve(VEC_pD&, ...) after this call diagnoses as it did before, and does not reuse the
    // supplied set. alg comes third, as in a call written (params, omittedTags, alg): a bool in
    // that position does not convert to Algorithm.
    // After std::bad_alloc or any other exception from a solve, destroy the System: constraints
    // may still point at SubSystem storage.
    // Obligations of a caller that uses this entry point (SESKAFEA C1.4b adapter):
    //  1. After Success call applySolution() and copy the values before any diagnose(): the
    //     confirmation solve inside diagnose() writes into the parameters and ends in
    //     resetToReference(), and an exception inside it can leave confirmation values in them.
    //  2. A diagnose() that throws std::bad_alloc, runs out of time or is cancelled is informational:
    //     report it as such, destroy the System, and never turn it into a failure of the solve (for
    //     example SolverMemory) or raise the memory cap for it. The diagnosis can need far more memory
    //     than the solve.
    //  3. Create only driving constraints with tag >= 0. Any other constraint puts its component on the
    //     two-SubSystem solve, which allocates dense xsize x xsize matrices.
    int solveWithSuppliedRedundancy(
        VEC_pD& params,
        const std::vector<int>& omittedTags,
        Algorithm alg = DogLeg,
        bool isFine = true,
        bool isRedundantsolving = false
    );
    int solve(
        SubSystem* subsys,
        bool isFine = true,
        Algorithm alg = DogLeg,
        bool isRedundantsolving = false
    );
    int solve(SubSystem* subsysA, SubSystem* subsysB, bool isFine = true, bool isRedundantsolving = false);

    void applySolution();
    void undoSolution();
    // FIXME: looks like XconvergenceFine is not the solver precision, at least in DogLeg
    // solver.
    //  Note: Yes, every solver has a different way of interpreting precision
    //  but one has to study what is this needed for in order to decide
    //  what to return (this is unchanged from previous versions)
    double getFinePrecision()
    {
        return convergence;
    }

    int diagnose(Algorithm alg = DogLeg);
    int dofsNumber() const
    {
        return hasDiagnosis ? dofs : -1;
    }
    void getConflicting(VEC_I& conflictingOut) const
    {
        conflictingOut = hasDiagnosis ? conflictingTags : VEC_I(0);
    }
    void getRedundant(VEC_I& redundantOut) const
    {
        redundantOut = hasDiagnosis ? redundantTags : VEC_I(0);
    }
    void getPartiallyRedundant(VEC_I& partiallyredundantOut) const
    {
        partiallyredundantOut = hasDiagnosis ? partiallyRedundantTags : VEC_I(0);
    }
    void getDependentParams(VEC_pD& pdependentparameterlist) const
    {
        pdependentparameterlist = pDependentParameters;
    }
    void getDependentParamsGroups(std::vector<std::vector<double*>>& pdependentparametergroups) const
    {
        pdependentparametergroups = pDependentParametersGroups;
    }
    bool isEmptyDiagnoseMatrix() const
    {
        return emptyDiagnoseMatrix;
    }

    bool hasConflicting() const
    {
        return !(hasDiagnosis && conflictingTags.empty());
    }
    bool hasRedundant() const
    {
        return !(hasDiagnosis && redundantTags.empty());
    }
    bool hasPartiallyRedundant() const
    {
        return !(hasDiagnosis && partiallyRedundantTags.empty());
    }

    void invalidatedDiagnosis();

    // Unit testing interface - not intended for use by production code
protected:
    size_t _getNumberOfConstraints(int tagID = -1)
    {
        if (tagID < 0) {
            return clist.size();
        }
        return std::count_if(clist.begin(), clist.end(), [tagID](Constraint* constraint) {
            return constraint->getTag() == tagID;
        });
    }
};


///////////////////////////////////////
// Helper elements
///////////////////////////////////////

void deleteAllContent(VEC_pD& doublevec);
void deleteAllContent(std::vector<Constraint*>& constrvec);
void deleteAllContent(std::vector<SubSystem*>& subsysvec);

#ifdef EIGEN_SPARSEQR_COMPATIBLE
// The Gauss-Newton step of DogLegGaussStep::SparseLeastNorm for a sparse J and residual fx. Declared for the
// unit tests that compare it with dense references; production code reaches it through solve_DL().
//   1. delta = 1e-9 max_i |J_i|^2, the same for every row. K = [[I, J^T], [J, -delta I]] is symmetric
//      quasi-definite (pivots +1 and -delta, no zero pivot) and is factored by SimplicialLDLT with a
//      fill-reducing order. An unknown or an equation shared by many rows is one node of K, not a dense
//      block: fill is O(nnz(J) + k (m + n)) for k such nodes. Nothing else is allocated at that scale.
//   2. h starts at 0. Each pass solves K [a; b] = [0; q] with q = -fx - J h and sets h += a. This is
//      h += J^T (J J^T + delta I)^-1 q, which contracts a singular-value direction sigma by
//      delta / (sigma^2 + delta) per pass and leaves the part of q in the null space of J^T (the
//      inconsistent part) out, so h stays in range(J^T). A uniform delta is what makes the limit the
//      unweighted least-squares step: J^T (J J^T + delta I)^-1 q = 0 if and only if J^T q = 0.
//      Passes stop when the correction of h is 1e-14 of h, or after twelve.
//   3. The result is the minimum-norm least-squares step to the accuracy of those passes. For full row
//      rank, or consistent rank deficiency, that is the least-norm or minimum-norm solution of J h = -fx.
//      Directions with sigma below about sqrt(delta) |J_i|max are damped, not inverted: on a nearly
//      singular J the step is Levenberg-like, and the DogLeg trust-region test accepts or shrinks it.
//   There is no QR fallback. The function returns true when the correction reached 1e-14 of h within the
//   twelve passes. A non-finite step is returned as zero. The step never allocates more than step 1.
bool sparseGaussNewtonStep(const Eigen::SparseMatrix<double>& J, const Eigen::VectorXd& fx, Eigen::VectorXd& h);
#endif

}  // namespace GCS

#endif  // PLANEGCS_GCS_H
