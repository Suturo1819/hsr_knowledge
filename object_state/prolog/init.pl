:- register_ros_package(knowrob_common).
:- register_ros_package(knowrob_objects).
:- register_ros_package(knowrob_actions).
:- register_ros_package(object_state).

:- rdf_db:rdf_register_ns(hsr_objects, 'http://www.semanticweb.org/suturo/ontologies/2018/10/objects#', [keep(true)]).


%:- use_module(library('prolog_object_state')).
%:- use_module(library('prolog_object_state_getter_setter')).
%:- use_module(library('prolog_object_state_util')).
%:- use_module(library('prolog_object_state_close')).
%:- use_module(library('test_calls')).
%:- use_module(library('srdl2')).
%:- use_module(library('knowrob_owl')).
%:- use_module(library('swrl')).
%:- use_module(library('owl_computable')).

:- owl_parse('package://knowrob_common/owl/knowrob.owl').
:- owl_parse('package://knowrob_common/owl/knowrob_common.owl').
:- owl_parse('package://object_state/owl/objects.owl').
