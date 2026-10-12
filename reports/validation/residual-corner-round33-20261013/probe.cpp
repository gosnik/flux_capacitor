#include "flux/core/copper_transaction.hpp"
#include "flux/core/geometry.hpp"
#include "flux/io/kicad_import.hpp"
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <numbers>
#include <optional>
#include <tuple>

using namespace flux;

std::optional<TrackArc> fillet(Point previous, Point corner, Point next, Nanometres maximum_radius) {
  const auto fx=static_cast<long double>(previous.x.value()-corner.x.value());
  const auto fy=static_cast<long double>(previous.y.value()-corner.y.value());
  const auto sx=static_cast<long double>(next.x.value()-corner.x.value());
  const auto sy=static_cast<long double>(next.y.value()-corner.y.value());
  const auto fl=std::hypot(fx,fy), sl=std::hypot(sx,sy), det=fx*sy-fy*sx;
  if (fl<=1 || sl<=1 || det==0) return std::nullopt;
  const auto fux=fx/fl, fuy=fy/fl, sux=sx/sl, suy=sy/sl;
  const auto cosine=std::clamp(fux*sux+fuy*suy,-1.L,1.L);
  const auto half=std::acos(cosine)/2, tan=std::tan(half), sine=std::sin(half);
  if(tan<=1e-12L||sine<=1e-12L)return std::nullopt;
  auto trim=static_cast<long double>(maximum_radius.value())/tan;
  trim=std::min({trim,fl*.45L,sl*.45L}); const auto radius=trim*tan;
  const auto bx=fux+sux,by=fuy+suy,bl=std::hypot(bx,by); if(trim<=1||radius<=1||bl<=1e-12L)return {};
  const auto cx=static_cast<long double>(corner.x.value()),cy=static_cast<long double>(corner.y.value());
  const auto cd=radius/sine, ox=cx+bx/bl*cd,oy=cy+by/bl*cd;
  const auto q=[](long double x,long double y){return Point{Nanometres{static_cast<std::int64_t>(std::llround(x))},Nanometres{static_cast<std::int64_t>(std::llround(y))}};};
  auto start=q(cx+fux*trim,cy+fuy*trim), end=q(cx+sux*trim,cy+suy*trim);
  const auto tx=cx-ox,ty=cy-oy,tl=std::hypot(tx,ty);
  auto mid=q(ox+tx/tl*radius,oy+ty/tl*radius);
  auto params=geometry::circular_arc_parameters(start,mid,end); if(!params) return {};
  const auto smooth=[](long double ax,long double ay,long double bx,long double by){ const auto product=std::hypot(ax,ay)*std::hypot(bx,by); return product>0 && ax*bx+ay*by>0 && std::abs(ax*by-ay*bx)/product<=.035L; };
  const auto sweep=params->sweep_radians>0?1.L:-1.L;
  const auto stx=-sweep*(static_cast<long double>(start.y.value())-params->center_y_nm), sty=sweep*(static_cast<long double>(start.x.value())-params->center_x_nm);
  const auto etx=-sweep*(static_cast<long double>(end.y.value())-params->center_y_nm), ety=sweep*(static_cast<long double>(end.x.value())-params->center_x_nm);
  if(!smooth(static_cast<long double>(start.x.value()-previous.x.value()),static_cast<long double>(start.y.value()-previous.y.value()),stx,sty) || !smooth(etx,ety,static_cast<long double>(next.x.value()-end.x.value()),static_cast<long double>(next.y.value()-end.y.value()))) return {};
  return TrackArc{ObjectId{"diagnostic-fillet"},NetId{},LayerId{},start,mid,end,Nanometres::from_mm(.25),false};
}
int main(int argc, char** argv){
 const auto path=std::filesystem::path("reports/validation/final-drc-parity-round32-20261013/tier2/failed-route-png/ft2232h_jtag/detailed-rejected/ft2232h_jtag.kicad_pcb");
 auto imported=io::import_kicad_pcb_file(path); if(!imported.board){std::cerr<<"import fail\n";return 2;} Board board=*imported.board;
 const auto x = argc > 1 ? std::strtod(argv[1], nullptr) : 131.1; const auto y = argc > 2 ? std::strtod(argv[2], nullptr) : 84.6; const Point corner{Nanometres::from_mm(x),Nanometres::from_mm(y)};
 for (const auto& footprint : board.footprints) for (const auto& pad : footprint.pads) if (pad.id.value() == "1d767ccc-83f5-48eb-b515-26d277ff587f") { const auto pos=geometry::transform_point(pad.local_position, footprint.position, footprint.rotation, footprint.side); std::cout << "target-pad id=" << pad.id.value() << " net=" << pad.net.value() << " pos=" << pos.x.value() << ',' << pos.y.value() << " shape=" << static_cast<int>(pad.shape) << " width=" << pad.width.value() << " height=" << pad.height.value() << " footprint=" << footprint.id.value() << "\n"; }
 std::vector<std::size_t> indexes; for(size_t i=0;i<board.tracks.size();++i) if(board.tracks[i].layer==LayerId{"F.Cu"}&&(board.tracks[i].start==corner||board.tracks[i].end==corner)) { indexes.push_back(i); std::cout << " match net=" << board.tracks[i].net.value() << " id=" << board.tracks[i].id.value() << "\n"; }
 std::cout<<"tracks="<<board.tracks.size()<<" arcs="<<board.track_arcs.size()<<" matches="<<indexes.size()<<"\n";
 if(indexes.size()!=2)return 3;
 auto a=board.tracks[indexes[0]],b=board.tracks[indexes[1]]; auto prev=a.start==corner?a.end:a.start, next=b.start==corner?b.end:b.start;
 for(int step=0;step<=8;++step){auto arc=fillet(prev,corner,next,Nanometres{1'000'000>>step}); if(!arc){std::cout<<"step="<<step<<" no_arc\n";continue;} arc->net=a.net; arc->layer=a.layer; Board probe=board; auto& ma=probe.tracks[indexes[0]]; auto& mb=probe.tracks[indexes[1]]; if(ma.end==corner)ma.end=arc->start;else ma.start=arc->start; if(mb.start==corner)mb.start=arc->end;else mb.end=arc->end; probe.track_arcs.push_back(*arc); CopperTransactionValidator val(board); auto report=val.validate(probe); std::cout<<"step="<<step<<" radius="<<(1'000'000>>step)<<" arc="<<arc->start.x.value()<<','<<arc->start.y.value()<<' '<<arc->middle.x.value()<<','<<arc->middle.y.value()<<' '<<arc->end.x.value()<<','<<arc->end.y.value()<<" structural="<<report.structural.issues.size()<<" pads="<<report.preserves_pad_components<<" conn="<<report.before_connectivity.open_connections<<"->"<<report.after_connectivity.open_connections<<" copper="<<report.copper.violations.size()<<"\n"; for(auto const& issue:report.structural.issues){std::cout<<"  STRUCT "<<issue.code<<" ids=";for(auto const&id:issue.object_ids)std::cout<<id<<",";std::cout<<"\n";} for(auto const& v:report.copper.violations){std::cout<<"  "<<v.code<<" actual="<<static_cast<double>(v.actual_distance_nm)<<" req="<<v.required_clearance.value()<<" ids=";for(auto const&id:v.object_ids)std::cout<<id<<',';std::cout<<"\n";}}
}
