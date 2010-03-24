%% Object Intersection

-module(wpc_intersect_object).
-export([init/0,menu/2,command/2]).
-include("wings.hrl").

-import(lists, [foldl/3,sort/1,reverse/1]).

init() ->
    true.

menu({body},Menu) ->
    reverse(parse(Menu, [], false));
menu(_,Menu) ->
    Menu.

parse([{_,weld,_,_}=A|Rest], NewMenu, false) ->
    parse(Rest, [menu_heading(),A|NewMenu], true);
parse([Elem|Rest], NewMenu, Found) ->
    parse(Rest, [Elem|NewMenu], Found);
parse([], NewMenu, true) ->
    NewMenu;
parse([], NewMenu, false) ->
    [menu_heading()|NewMenu].

menu_heading() ->
    {?__(1,"Intersect"),intersect,?__(2,"Intersection of two meshes")}.

command({body,intersect}, St) ->
    intersect(St);
command(_, _) -> next.

%%%
%%% Intersection of two objects.
%%%

intersect(#st{shapes=Shapes0,sel=[{IdA,_},{IdB,_}]}=St) ->
    WeA0 = gb_trees:get(IdA, Shapes0),
    WeB0 = gb_trees:get(IdB, Shapes0),
    {WeA1,FacePosA,NewEdgesA, WeB1,NewEdgesB,FacePosB} = inspect_mesh(WeA0, WeB0),
    WeA2 = final_cut(NewEdgesA, WeA1, FacePosB),
    WeB2 = final_cut(NewEdgesB, WeB1, FacePosA),
    NewEsA = wings_we:new_items_as_ordset(edge, WeA1, WeA2),
    NewEsB = wings_we:new_items_as_ordset(edge, WeB1, WeB2),
    ShortEdgesA = [Edge || Edge <- NewEsA, short_edge(Edge, WeA2)],
    ShortEdgesB = [Edge || Edge <- NewEsB, short_edge(Edge, WeB2)],
    WeA = wings_collapse:collapse_edges(ShortEdgesA, WeA2),
    WeB = wings_collapse:collapse_edges(ShortEdgesB, WeB2),
    Shapes1 = gb_trees:update(IdA, WeA, Shapes0),
    Shapes = gb_trees:update(IdB, WeB, Shapes1),
    SelA = wings_we:new_items_as_gbset(vertex, WeA0, WeA),
    SelB = wings_we:new_items_as_gbset(vertex, WeB0, WeB),
    St#st{selmode=vertex,sel=[{IdA,SelA},{IdB,SelB}],shapes=Shapes};
intersect(_) ->
    wings_u:error(?__(1,"Selection must be of two objects.")).

short_edge(Edge, #we{es=Etab,vp=Vtab}) ->
    #edge{vs=Va,ve=Vb} = array:get(Edge, Etab),
    VaPos = array:get(Va, Vtab),
    VbPos = array:get(Vb, Vtab),
    e3d_vec:dist(VaPos, VbPos) < 1.0E-3.


-record(int,
    {fs=orddict:new(), %% [{Face,Pos}] Positions to add inside faces of the plane object
     epos=orddict:new(), %% [{Edge,Pos}] to cut on the intersecting object
     es=[], %% [{Edge,Pos}] Positions to add to edges of the plane object
     vs=[], %% [{Vert,Pos}] Positions to add to vertices of the plane object
     ecuts=gb_sets:new(), %% gb_set - Edges that pass through 2 faces
     tes=[], %% gb_set - Faces to tesselate
     fix=false}). %% fix mode becomes enabled when mesh needs adjustments.
                %% this reduces the amount of processing since the mesh will
                %% have to be rescaned after mesh preparing cuts are made anyway.


inspect_mesh(#we{fs=FtabA}=WeA0, #we{fs=FtabB}=WeB0) ->
    FacesA = gb_trees:to_list(FtabA),
    case inspect(FacesA, WeA0, WeB0, #int{}) of
      #int{fix=false}=IntB ->
        FacesB = gb_trees:to_list(FtabB),
        case inspect(FacesB, WeB0, WeA0, #int{}) of
          #int{fix=false}=IntA ->
            finish(IntA, IntB, WeA0, WeB0);
          #int{tes=Tes,ecuts=Es,epos=Epos,fix=true} ->
            WeB = tesselate(Tes, WeB0),
            WeA = cut_edge_faces(Es, Epos, WeA0),
            inspect_mesh(WeA, WeB)
        end;
      #int{tes=Tes,ecuts=Es,epos=Epos,fix=true} ->
        WeA = tesselate(Tes, WeA0),
        WeB = cut_edge_faces(Es, Epos, WeB0),
        inspect_mesh(WeA, WeB)
    end.

finish(#int{fs=FsA,es=EsA,vs=VsA,epos=EposA},
       #int{fs=FsB,es=EsB,vs=VsB,epos=EposB}, WeA0, WeB0) ->
    EdgesA = lists:usort(EposA ++ EsB),
    EdgesB = lists:usort(EposB ++ EsA),
    {NewEdgesA,WeA} = cut_intersecting_edges(EdgesA, VsB, WeA0),
    {NewEdgesB,WeB} = cut_intersecting_edges(EdgesB, VsA, WeB0),
    {WeA,FsA,NewEdgesA,WeB,NewEdgesB,FsB}.
    
cut_intersecting_edges(EdgePos, Vs0,  We0) ->
    {Vs1,We1} = cut_edges(EdgePos, We0),
    Vs = lists:usort(Vs1 ++ Vs0),
    We = wings_vertex_cmd:connect(Vs, We1),
    NewEdges = vs_to_edges(Vs, We, []),
    {NewEdges,We}.

vs_to_edges([Va|Vs], We, Acc0) ->
%% finds the edges between any two listed vertices and returns a gb_set fo edges
    Edges = wings_vertex:fold(fun(Edge, _, EdgeRec, Acc) ->
         Vb = wings_vertex:other(Va, EdgeRec),
         case ordsets:is_element(Vb,Vs) of
           true -> [Edge|Acc];
           _ -> Acc
         end
     end, Acc0, Va, We),
    vs_to_edges(Vs, We, Edges);
vs_to_edges([], _, Edges) ->
    lists:usort(Edges).

cut_edges(Es, We0) ->
    lists:mapfoldl(fun
        ({Edge,default}, We1) ->
            {We,V} = wings_edge:fast_cut(Edge, default, We1),
            {V,We};
        ({Edge,Pos}, #we{es=Etab,vp=Vtab}=We1) ->
            #edge{vs=Va,ve=Vb} = array:get(Edge, Etab),
            PosA = array:get(Va, Vtab),
            case e3d_vec:dist(PosA, Pos) < 1.0E-6 of
              true -> {Va,We1};
              false ->
                PosB = array:get(Vb, Vtab),
                case e3d_vec:dist(PosB, Pos) < 1.0E-6 of
                  true -> {Vb,We1};
                  false ->
                    {We,V} = wings_edge:fast_cut(Edge, Pos, We1),
                    {V,We}
                end
            end
         end, We0, Es).

cut_edge_faces(Es, EdgePos, We0) ->
    find_elem2cut_0(Es, EdgePos, We0, orddict:new(), []).

find_elem2cut_0(Es, EdgePos, #we{es=Etab}=We0, EAcc0, VAcc0) ->
    case gb_sets:is_empty(Es) of
      true ->
        {Vs,We1} = cut_edges(EAcc0, We0),
        wings_vertex_cmd:connect(Vs ++ VAcc0, We1);
      false ->
        {Edge, Edges} = gb_sets:take_smallest(Es),
        #edge{lf=Lf,rf=Rf} = array:get(Edge, Etab),
        Pos = fetch_edge_pos(orddict:fetch(Edge, EdgePos)),
        EAcc1 = orddict:store(Edge, Pos, EAcc0),
        {EAcc,VAcc} = find_elem2cut([Lf,Rf], EdgePos, Edge, We0, EAcc1, VAcc0),
        find_elem2cut_0(Edges, EdgePos, We0, EAcc, VAcc)
    end.

fetch_edge_pos(Pos) when is_list(Pos) -> e3d_vec:average(Pos);
fetch_edge_pos(Pos) -> Pos.

find_elem2cut([Face|Faces], EdgePos, Edge, We0, EAcc0, VAcc0) ->
    Len = length(wings_face:to_vertices([Face], We0)),
    N = Len div 2,
    {EAcc,VAcc} = case Len rem 2 of
      0 ->
        {E,_} = next_edge(Edge, Face, We0, N),
        EAcc1 = case orddict:find(E, EdgePos) of
          error -> orddict:store(E, default, EAcc0);
          {ok,P} ->
            Pos = fetch_edge_pos(P),
            orddict:store(E, Pos, EAcc0)
        end,
        {EAcc1,VAcc0};
      1 ->
        {_,V} = next_edge(Edge, Face, We0, N+1),
        {EAcc0,[V|VAcc0]}
    end,
    find_elem2cut(Faces, EdgePos, Edge, We0, EAcc, VAcc);
find_elem2cut([], _, _, _, EAcc, VAcc) ->
    {EAcc,VAcc}.

next_edge(Edge, Face, #we{es=Etab},1)->
    case array:get(Edge, Etab) of
        #edge{lf=Face,ltsu=NextEdge,vs=V} -> {NextEdge,V};
        #edge{rf=Face,rtsu=NextEdge,ve=V} -> {NextEdge,V}
    end;
next_edge(Edge, Face, #we{es=Etab}=We,N)->
    NEdge = case array:get(Edge, Etab) of
        #edge{lf=Face,ltsu=NextEdge} -> NextEdge;
        #edge{rf=Face,rtsu=NextEdge} -> NextEdge
    end,
    next_edge(NEdge, Face, We, N-1).

inspect([{Face,E}|Fs], WeA0, #we{es=Etab,vp=Vtab}=WeB0, Int0) ->
    {FaceCenter,FaceNorm,FaceEdges} = face_data(Face, E, WeA0),
    SideArray = assign_side(Vtab, FaceNorm, FaceCenter),
    Int = array:sparse_foldl(fun
      (Edge, #edge{vs=Va,ve=Vb}, Int1) ->
        {SideA,PosA} = array:get(Va, SideArray),
        {SideB,PosB} = array:get(Vb, SideArray),
        %% Check if Edge's vs are on either side of the Face
        case oposite_sides(SideA, SideB) of
          false -> Int1;
          true ->
            %% Here we are checking to see if the Face Center is actually inside
            %% the Face. If it isn't, the the face is C or L shaped and needs to
            %% be tesselated.
            %% Return false if the face is C or L shaped
            case point_inside_face(FaceEdges, FaceNorm, Face, FaceCenter, WeA0) of
              true ->
                EdgeVec = e3d_vec:norm_sub(PosA, PosB),
                Point = intersect_vec_plane(FaceCenter, PosA, FaceNorm, EdgeVec),
                case point_inside_face(FaceEdges, FaceNorm, Face, Point, WeA0) of
                  false -> Int1;
                  {Type,Id} ->
                    case store_edge(Edge, Point, Int1) of
                      #int{fix=true}=Int2 -> Int2;
                      #int{es=ObjEs}=Int2 when Type =:= on_edge ->
                        Int2#int{es=[{Id,Point}|ObjEs]};
                      #int{vs=ObjVs}=Int2 when Type =:= on_vertex ->
                        Int2#int{vs=[Id|ObjVs]}
                    end;
                  true ->
                    case store_edge(Edge, Point, Int1) of
                      #int{fix=true}=Int2 -> Int2;
                      Int2 ->
                        store_face(Face, Point, Int2)
                    end;
              _ ->
                #int{tes=Tes} = Int1,
                N = length(FaceEdges),
                Int1#int{tes=[{Face,N}|Tes],fix=true}
                end
            end
        end
    end, Int0, Etab),
    inspect(Fs, WeA0, WeB0, Int);
inspect([], _, _, Int) -> Int.

store_face(Face, Point, #int{fs=ObjFs0}=Int0) ->
    case orddict:find(Face, ObjFs0) of
      error ->
        ObjFs =  orddict:store(Face, [Point], ObjFs0),
        Int0#int{fs=ObjFs};
      {ok,Prev} ->
        ObjFs = orddict:store(Face, [Point|Prev], ObjFs0),
        Int0#int{fs=ObjFs}
    end.

store_edge(Edge, Point, #int{epos=Epos0,ecuts=Ec}=Int0) ->
    case orddict:find(Edge, Epos0) of
      error ->
        Epos =  orddict:store(Edge, Point, Epos0),
        Int0#int{epos=Epos};
      {ok,Point} ->
        Int0;
      {ok,Prev} when is_list(Prev) ->
        Epos = orddict:store(Edge, [Point|Prev], Epos0),
        Int0#int{epos=Epos};
      {ok,Prev} ->
        Epos = orddict:store(Edge, [Point,Prev], Epos0),
        Int0#int{epos=Epos,ecuts=gb_sets:insert(Edge,Ec),fix=true}
     end.

%%%%%%%%%%%%%%%%%%%%%
face_data(Face, Edge, #we{es=Etab,vp=Vtab}) ->
    face_data_1(Edge, Etab, Vtab, Face, Edge, [], []).

face_data_1(LastEdge, _, _, _, LastEdge, PosAcc, EsAcc) when PosAcc =/= [] ->
    FaceCenter = e3d_vec:average(PosAcc),
    FaceNorm = e3d_vec:normal(PosAcc),
    {FaceCenter,FaceNorm,EsAcc};
face_data_1(Edge, Etab, Vtab, Face, LastEdge, PosAcc, EsAcc) ->
    case array:get(Edge, Etab) of
      #edge{vs=V,lf=Face,ltsu=NextEdge} ->
          Pos = array:get(V, Vtab),
          face_data_1(NextEdge, Etab, Vtab, Face, LastEdge, [Pos|PosAcc], [Edge|EsAcc]);
      #edge{ve=V,rf=Face,rtsu=NextEdge} ->
          Pos = array:get(V, Vtab),
          face_data_1(NextEdge, Etab, Vtab, Face, LastEdge, [Pos|PosAcc], [Edge|EsAcc])
    end.

tesselate(Faces0, We0) ->
    Gb = gb_sets:new(),
    {Tfs,Qfs} = foldl(fun
      ({F,4}, {TrFs,QdFs}) -> {gb_sets:add(F, TrFs),QdFs};
      ({F,_}, {TrFs,QdFs}) -> {TrFs,gb_sets:add(F, QdFs)}
    end, {Gb,Gb}, Faces0),
    We = wings_tesselation:triangulate(Tfs,We0),
    wings_tesselation:quadrangulate(Qfs,We).


point_inside_face([Edge|Edges], FaceNorm, Face, Point, We) ->
    {PosA,PosB,Va,Vb} = edge_pos_dir(Edge, Face, We),
    EdgeVec = e3d_vec:sub(PosA, PosB),
    PointVecA = e3d_vec:sub(PosA, Point),
    PointVecB = e3d_vec:sub(PosB, Point),
    case point_on_edge(EdgeVec, PointVecA, PointVecB) of
      vertex_a -> {on_vertex,Va};
      vertex_b -> {on_vertex,Vb};
      on_edge -> {on_edge,Edge};
      false ->
        case inside_face_dot(EdgeVec, PointVecA, FaceNorm) > 0 of
          true -> point_inside_face(Edges, FaceNorm, Face, Point, We);
          false -> false
        end
    end;
point_inside_face([],_,_,_,_) -> true.

inside_face_dot(EdgeVec, PointVec, FaceNorm) ->
    CrossVec = e3d_vec:norm(e3d_vec:cross(EdgeVec, FaceNorm)),
    e3d_vec:dot(CrossVec, e3d_vec:norm(PointVec)).

point_on_edge(EdgeVec, PointVecA, PointVecB) ->
    Total0 = math:sqrt(sqr_length(EdgeVec)),
    LenA = math:sqrt(sqr_length(PointVecA)),
    LenB = math:sqrt(sqr_length(PointVecB)),
    Total1 = LenA + LenB,
    case Total0 < Total1 of
      true -> false;
      false ->
        case (Total0 - Total1) < 1.0E-4 of
          true ->
            if LenA < 1.0E-4 -> vertex_a;
               LenB < 1.0E-4 -> vertex_b;
               true -> on_edge
            end;
          false -> false
        end
    end.

sqr_length({X,Y,Z}) ->
    X*X+Y*Y+Z*Z.

edge_pos_dir(Edge, Face, #we{es=Etab,vp=Vtab}) ->
    case array:get(Edge, Etab) of
      #edge{rf=Face,vs=Va,ve=Vb} ->
        PosA = array:get(Va, Vtab),
        PosB = array:get(Vb, Vtab);
      #edge{lf=Face,vs=Vb,ve=Va} ->
        PosA = array:get(Va, Vtab),
        PosB = array:get(Vb, Vtab)
    end,
    {PosA,PosB,Va,Vb}.

oposite_sides(A, B) when A =:= on_vertex; B =:= on_vertex -> true;
oposite_sides(Side, Side) -> false;
oposite_sides(_, _) -> true.

assign_side(Vtab, Plane, CutPoint) ->
%% Create an array using each vertex id as the index and {Dot < 0, Pos} as the
%% value. The Dot boolean is later used to determine the side of the plane on
%% which the vertex resides. The position is stored just so is doesn't have to
%% be looked up again.
    array:sparse_foldl(fun
      (V, Pos, Array) ->
        Vec = e3d_vec:norm_sub(Pos, CutPoint),
        Dot = e3d_vec:dot(Vec, Plane),
        case Vec =:= e3d_vec:zero() of
          true ->
            array:set(V, {on_vertex ,Pos}, Array);
          false ->
            array:set(V, {Dot < 0 ,Pos}, Array)
        end
    end, Vtab, Vtab).

intersect_vec_plane(PosA, PosB, FaceNorm, EdgeVec) ->
%% Return point where Vector through PosA intersects with plane at PosB
    case e3d_vec:dot(EdgeVec,FaceNorm) of
      0.0 ->
        Intersection = e3d_vec:dot(e3d_vec:sub(PosB, PosA), FaceNorm),
        e3d_vec:add(PosB, e3d_vec:mul(FaceNorm, Intersection));
      Dot ->
        Intersection = e3d_vec:dot(e3d_vec:sub(PosA, PosB), FaceNorm) / Dot,
        e3d_vec:add(PosB, e3d_vec:mul(EdgeVec, Intersection))
    end.


%%%%%%%%%%%%%%%%%%%%

final_cut([Edge|Edges], #we{es=Etab,vp=Vtab}=We0, FacePos) ->
    #edge{lf=Lf,rf=Rf,vs=Va,ve=Vb} = array:get(Edge, Etab),
    We = case orddict:find(Lf, FacePos) of
      {ok,Pos} ->
        PosA = array:get(Vb, Vtab),
        PosB = array:get(Va, Vtab),
        final_cut_0(Edge, PosA, PosB, Pos, We0);
      error ->
        case orddict:find(Rf, FacePos) of
          {ok,Pos} ->
            PosA = array:get(Va, Vtab),
            PosB = array:get(Vb, Vtab),
            final_cut_0(Edge, PosA, PosB, Pos, We0);
          error ->
            We0
        end
    end,
    final_cut(Edges, We, FacePos);
final_cut([], We, _) ->
    We.

final_cut_0(Edge, PosA, PosB, NewPos, We0) ->
    case NewPos of
      [Pos] ->
        %% single cut
        {We1,_} = wings_edge:fast_cut(Edge, Pos, We0),
        We1;
      Positions ->
        %% multiple cuts
        final_cut_2(Positions, Edge, PosA, PosB, We0)
    end.

final_cut_2(Positions, Edge, PosA, PosB, We0) ->
%% Once we have varified the positions for this Edge, run this function
    {DistListA,DistListB} = foldl(fun(P, {AccA,AccB}) ->
            DistA = e3d_vec:dist(P, PosA),
            DistB = e3d_vec:dist(P, PosB),
            {[{DistA,P}|AccA],[{DistB,P}|AccB]}
        end, {[],[]}, Positions),
    DpA = sort(DistListA),
    DpB = sort(DistListB),
    DistList = final_cut_distlist(DpA, DpB, [], []),
    foldl(fun(P, We1) ->
            {We2,_} = wings_edge:fast_cut(Edge, P, We1),
            We2
    end, We0, DistList).

final_cut_distlist([{_,Pa}|FromVa], [{_,Pb}|FromVb], AccA0, AccB) ->
    AccA = [Pa|AccA0],
    case lists:member(Pb,AccA) of
      true ->
        reverse(reverse(AccA) ++ AccB);
      false -> final_cut_distlist(FromVa,FromVb,AccA,[Pb|AccB])
    end;
final_cut_distlist([],[], AccA,_) -> AccA.
