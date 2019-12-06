:- register_ros_package(knowrob_common).
:- register_ros_package(knowrob_objects).
:- register_ros_package(knowrob_actions).
:- register_ros_package(object_state).
:- register_ros_package(srdl).
:- register_ros_package(knowrob_vis).
:- register_ros_package(knowrob_memory).

:- rdf_db:rdf_register_ns(hsr_objects, 'http://www.semanticweb.org/suturo/ontologies/2018/10/objects#', [keep(true)]).
:- rdf_db:rdf_register_ns(robocup, 'http://knowrob.org/kb/robocup.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(srdl2_comp, 'http://knowrob.org/kb/srdl2-comp.owl#', [keep(true)]).

:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/owl_parser')).
:- use_module(library('semweb/owl')).
:- use_module(library('knowrob/computable')).
:- use_module(library('knowrob/temporal')).
:- use_module(library('knowrob/beliefstate')).
:- use_module(library('knowrob/transforms')).
%:- use_module(library('knowrob/owl')).
%:- use_module(library('knowrob/knowrob/rdfs')).
:- use_module(library('knowrob/vis')).
%:- use_module(library('knowrob/comp_spatial')).
%:- use_module(library('knowrob/owl_export')).

:- use_module(library('object_state')).
:- use_module(library('surfaces')).
:- use_module(library('beliefstate')).
:- use_module(library('spatial_comp')).
%:- use_module(library('static_preset')).

:- owl_parse('package://knowrob_common/owl/knowrob.owl').
:- owl_parse('package://object_state/owl/objects.owl').
