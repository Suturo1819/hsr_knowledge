:- module(spatial_comp,
    [
      surface_size/2,
      srdl_matrix/2,
      matrix_multiply/3
    ]).

:- rdf_db:rdf_register_ns(hsr_objects, 'http://www.semanticweb.org/suturo/ontologies/2018/10/objects#', [keep(true)]).
:- rdf_db:rdf_register_ns(robocup, 'http://knowrob.org/kb/robocup.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(srdl2_comp, 'http://knowrob.org/kb/srdl2-comp.owl#', [keep(true)]).

:- rdf_meta
    surface_size(r,?),
    srdl_matrix(r,?),
    matrix_multiply(r,r,?).

surface_size(TFFrame, Size) :-
    rdf_has(Instance, srdl2_comp:'urdfName', literal(TFFrame)),
    rdf_has(Instance, srdl2_comp:'box_size', Size), !.

surface_size(Instance, Size) :-
    rdf_has(Instance, srdl2_comp:'box_size', Size).

% for matrices with 3 predecessor joints
srdl_matrix(SrdlObj, Matrix) :-
    rdf_has(Joint, srdl2_comp:'succeedingLink', SrdlObj),
    rdf_has(Proprioception, knowrob:'objectActedOn', Joint),
    rdf_has(Proprioception, knowrob:'eventOccursAt', Loc),
    rdf_has(Loc, knowrob:'relativeTo' , RelLoc),
    rdf_has(RelLoc, knowrob:'relativeTo' , RelRelLoc),
    transform_data(Loc, (Trans, Rot)),
    transform_data(RelLoc, (RelTrans, RelRot)),
    matrix(LocMatrix, Trans, Rot),
    matrix(RelMatrix, RelTrans, RelRot),
    matrix_multiply(LocMatrix, RelMatrix, TempMatrix),
    transform_data(RelRelLoc, (RelRelTrans, RelRelRot)),
    matrix(RelRelMatrix, RelRelTrans, RelRelRot),
    matrix_multiply(TempMatrix, RelRelMatrix, Matrix), !.


% for matrices with 2 predecessor joints
srdl_matrix(SrdlObj, Matrix) :-
    rdf_has(Joint, srdl2_comp:'succeedingLink', SrdlObj),
    rdf_has(Proprioception, knowrob:'objectActedOn', Joint),
    rdf_has(Proprioception, knowrob:'eventOccursAt', Loc),
    rdf_has(Loc, knowrob:'relativeTo' , RelLoc),
    transform_data(Loc, (Trans, Rot)),
    transform_data(RelLoc, (RelTrans, RelRot)),
    matrix(LocMatrix, Trans, Rot),
    matrix(RelMatrix, RelTrans, RelRot),
    matrix_multiply(LocMatrix, RelMatrix, Matrix).

matrix_multiply(Matrix, Transform, OutMatrix) :-
    matrix(Matrix, _, Rot),
    matrix(Transform, TTrans, TRot),
    matrix_translate(Matrix, TTrans, TranslatedMat),
    quaternion_multiply(Rot, TRot, TranslatedRot),
    matrix(TranslatedMat, TranslatedTrans, _),
    matrix(OutMatrix, TranslatedTrans, TranslatedRot).

%recursive_relative_matrix(Loc, Matrix) :-
%    (rdf_has(Loc, knowrob:'relativeTo' , RelLoc),
%        recursive_relative_matrix(RelLoc, RelMatrix),
%        transform_data(Loc, (Trans, Rot)),
%        matrix(LocMatrix, Trans, Rot),
%        matrix_multiply(LocMatrix, RelMatrix, Matrix)
%        );
%        Matrix = [1.0,0.0,0.0,0.0,
%                  0.0,1.0,0.0,0.0,
%                  0.0,0.0,1.0,0.0,
%                  0.0,0.0,0.0,1.0].
%
%
%recursive_relative_matrix(_, Matrix) :-
%    Matrix = [1,0,0,0,
%               0,1,0,0,
%               0,0,1,0,
%               0,0,0,1].

