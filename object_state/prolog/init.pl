:- register_ros_package(knowrob_common).
:- register_ros_package(knowrob_objects).
:- register_ros_package(knowrob_actions).
:- register_ros_package(object_state).
:- register_ros_package(knowrob_srdl).


:- rdf_db:rdf_register_ns(hsr_objects, 'http://www.semanticweb.org/suturo/ontologies/2018/10/objects#', [keep(true)]).
%:- rdf_db:rdf_register_ns(dul, 'http://www.ontologydesignpatterns.org/ont/dul/DUL.owl#', [keep(true)]).

:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/owl_parser')).
:- use_module(library('semweb/owl')).
:- use_module(library('knowrob/computable')).
:- use_module(library('knowrob/temporal')).
:- use_module(library('knowrob/owl')).

:- use_module(library('object_state')).

:- owl_parse('package://knowrob_common/owl/knowrob.owl').
%:- owl_parse('package://knowrob_common/owl/knowrob_common.owl').
:- owl_parse('package://object_state/owl/objects.owl').
