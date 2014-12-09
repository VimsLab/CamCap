#include "stdafx.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <boost/filesystem.hpp>
#include <iostream>
#include <algorithm>
#include <numeric>
#include <ctime>
#include <iomanip>
#include <map>
#include <bitset>
#include <tbb/compat/ppl.h>
#include <tbb/task_scheduler_init.h>
#include <tbb/concurrent_vector.h>
#include <tbb/flow_graph.h>
#include "TriggeredCam.h"

using namespace std;
using namespace cv;
using namespace FlyCapture2;
using namespace Concurrency;
using namespace tbb;
using namespace boost::filesystem;
using namespace tbb::flow;

#define PG_CAMS (0)
#define XC_CAMS (2)
#define PG1394_PRI_CAMS (1)
#define PG1394_SEC_CAMS (1)
#define N_CAMS (PG_CAMS + PG1394_PRI_CAMS + PG1394_SEC_CAMS + XC_CAMS)

/*
templated serial for-loop helper
*/
template <typename Index, typename Function>
void serial_for(Index first, Index last, const Function& f){
	while (first != last) {
		f(first);
		++first;
	}
}

/*
flags representing keys pressed in GUI
*/
typedef enum
{
	CONTINUE = 0ULL,
	QUIT = 1ULL,
	DISPLAY = 2ULL,
	SAVE = 4ULL
} WaitKey;

/*
limits fps and retrieves flags from keys pressed in GUI
*/
WaitKey waitKey(double fps, double elapsed){
	int key = waitKey(int(round(max(1., 1000. * (1. / fps - elapsed)))));
	switch (key)
	{
	case 'q':
	case 'Q':
		return WaitKey::QUIT;
	case 'd':
	case 'D':
		return WaitKey::DISPLAY;
	case 's':
	case 'S':
		return WaitKey::SAVE;
	default:
		return WaitKey::CONTINUE;
	}
}

/*
triggered frame structure passed between nodes
*/
struct TriggeredFrame{
	uint64_t flags;
	int frame_no;
	uint32_t serial;
	Mat frame;
	static int getTag(const TriggeredFrame& f){
		return f.frame_no;
	};
};

/*
templated helper for supporting 1-4 triggered cameras
*/
template<size_t DIM>
struct TFHelper;
template<>
struct TFHelper < 1 > {
	typedef tuple<TriggeredFrame > TFtuple;
	typedef join_node<TFtuple> Multiplexer;
	typedef multifunction_node<TriggeredFrame, TFtuple> Normalizer;
	typedef tuple_element<0, Normalizer::output_ports_type >::type Normalizer_output_port_type;
	static Multiplexer getMultiplexer(graph& g){
		return Multiplexer(g);
	}
	static Normalizer_output_port_type& getOutputPort(const size_t switch_on, Normalizer::output_ports_type& op){
		switch (switch_on)
		{
		case 0:
			return get<0>(op);
		default:
			throw exception("invalid switch_on argument");
		}
	}
	static void make_edges(Normalizer& normalizer, Multiplexer& multiplexer){
		make_edge(output_port<0>(normalizer), input_port<0>(multiplexer));
	}
	static vector<Mat> getFrames(const TFtuple& tp){
		vector<Mat> r(1);
		r[0] = get<0>(tp).frame;
		return r;
	}
};
template<>
struct TFHelper < 2 > {
	typedef tuple<TriggeredFrame, TriggeredFrame> TFtuple;
	typedef join_node<TFtuple, tag_matching> Multiplexer;
	typedef multifunction_node<TriggeredFrame, TFtuple> Normalizer;
	typedef tuple_element<0, Normalizer::output_ports_type >::type Normalizer_output_port_type;
	static Multiplexer getMultiplexer(graph& g){
		return Multiplexer(g, TriggeredFrame::getTag, TriggeredFrame::getTag);
	}
	static Normalizer_output_port_type& getOutputPort(const size_t switch_on, Normalizer::output_ports_type& op){
		switch (switch_on)
		{
		case 0:
			return get<0>(op);
		case 1:
			return get<1>(op);
		default:
			throw exception("invalid switch_on argument");
		}
	}
	static void make_edges(Normalizer& normalizer, Multiplexer& multiplexer){
		make_edge(output_port<0>(normalizer), input_port<0>(multiplexer));
		make_edge(output_port<1>(normalizer), input_port<1>(multiplexer));
	}
	static vector<Mat> getFrames(const TFtuple& tp){
		vector<Mat> r(2);
		r[0] = get<0>(tp).frame;
		r[1] = get<1>(tp).frame;
		return r;
	}
};

template<>
struct TFHelper < 3 > {
	typedef tuple<TriggeredFrame, TriggeredFrame, TriggeredFrame> TFtuple;
	typedef join_node<TFtuple, tag_matching> Multiplexer;
	typedef multifunction_node<TriggeredFrame, TFtuple> Normalizer;
	typedef tuple_element<0, Normalizer::output_ports_type >::type Normalizer_output_port_type;
	static Multiplexer getMultiplexer(graph& g){
		return Multiplexer(g, TriggeredFrame::getTag, TriggeredFrame::getTag, TriggeredFrame::getTag);
	}
	static Normalizer_output_port_type& getOutputPort(const size_t switch_on, Normalizer::output_ports_type& op){
		switch (switch_on)
		{
		case 0:
			return get<0>(op);
		case 1:
			return get<1>(op);
		case 2:
			return get<2>(op);
		default:
			throw exception("invalid switch_on argument");
		}
	}
	static void make_edges(Normalizer& normalizer, Multiplexer& multiplexer){
		make_edge(output_port<0>(normalizer), input_port<0>(multiplexer));
		make_edge(output_port<1>(normalizer), input_port<1>(multiplexer));
		make_edge(output_port<2>(normalizer), input_port<2>(multiplexer));
	}
	static vector<Mat> getFrames(const TFtuple& tp){
		vector<Mat> r(3);
		r[0] = get<0>(tp).frame;
		r[1] = get<1>(tp).frame;
		r[2] = get<2>(tp).frame;
		return r;
	}
};
template<>
struct TFHelper < 4 > {
	typedef tuple<TriggeredFrame, TriggeredFrame, TriggeredFrame, TriggeredFrame> TFtuple;
	typedef join_node<TFtuple, tag_matching> Multiplexer;
	typedef multifunction_node<TriggeredFrame, TFtuple> Normalizer;
	typedef tuple_element<0, Normalizer::output_ports_type >::type Normalizer_output_port_type;
	static Multiplexer getMultiplexer(graph& g){
		return Multiplexer(g, TriggeredFrame::getTag, TriggeredFrame::getTag, TriggeredFrame::getTag, TriggeredFrame::getTag);
	}
	static Normalizer_output_port_type& getOutputPort(const size_t switch_on, Normalizer::output_ports_type& op){
		switch (switch_on)
		{
		case 0:
			return get<0>(op);
		case 1:
			return get<1>(op);
		case 2:
			return get<2>(op);
		case 3:
			return get<3>(op);
		default:
			throw exception("invalid switch_on argument");
		}
	}
	static void make_edges(Normalizer& normalizer, Multiplexer& multiplexer){
		make_edge(output_port<0>(normalizer), input_port<0>(multiplexer));
		make_edge(output_port<1>(normalizer), input_port<1>(multiplexer));
		make_edge(output_port<2>(normalizer), input_port<2>(multiplexer));
		make_edge(output_port<3>(normalizer), input_port<3>(multiplexer));
	}
	static vector<Mat> getFrames(const TFtuple& tp){
		vector<Mat> r(4);
		r[0] = get<0>(tp).frame;
		r[1] = get<1>(tp).frame;
		r[2] = get<2>(tp).frame;
		r[3] = get<3>(tp).frame;
		return r;
	}
};

// right now, we only implement writing and diplaying so this node outputs to 2 functions
typedef multifunction_node<TriggeredFrame, TFHelper<2>::TFtuple > Dispatcher;

int main_graph(int argc, char **argv){
	const uint32_t pg_serials[] = { 12010990, 12010988 };
	const uint32_t pg1394_pri_serials[] = { 13020556 };
	const uint32_t pg1394_sec_serials[] = { 13232653 };
	const uint32_t xc_serials[] = { 5003, 5270 };
	const string winname = "stream";

	//configurable params
	const int framecount = -1; //-1 means infinity
	const double fps = 16;

	//initialize cameras
	vector<Ptr<TriggeredCam> > cams;
	for (const uint32_t *serial = pg_serials; serial != pg_serials + PG_CAMS;
		serial++) {
		cerr << "init: " << *serial << endl;
		cams.push_back(new PGTriggeredCam(*serial));
	}
	for (const uint32_t *serial = pg1394_pri_serials; serial != pg1394_pri_serials + PG1394_PRI_CAMS;
		serial++) {
		cerr << "init: " << *serial << endl;
		cams.push_back(new PG1394TriggeredCam(*serial, true));
	}
	for (const uint32_t *serial = pg1394_sec_serials; serial != pg1394_sec_serials + PG1394_SEC_CAMS;
		serial++) {
		cerr << "init: " << *serial << endl;
		cams.push_back(new PG1394TriggeredCam(*serial, false));
	}
	for (const uint32_t *serial = xc_serials; serial != xc_serials + XC_CAMS;
		serial++) {
		cerr << "init: " << *serial << endl;
		cams.push_back(new XCTriggeredCam(*serial));
	}
	assert_throw(cams.size() > 0);

	//create output path
	const time_t timestamp = time(0);
	path basePath;
	basePath += "data";
	basePath += path::preferred_separator;
	basePath += std::to_string(timestamp);
	basePath += path::preferred_separator;
	for (int i = 0; i < cams.size(); i++){
		path camPath = basePath;
		camPath += std::to_string(cams[i]->serial);
		camPath += path::preferred_separator;
		create_directories(camPath);
	}

	// initialize threads and graph flow
	task_scheduler_init();
	graph g;

	/*
	dispatches triggered frames based to nodes for further processing based on flags
	*/
	Dispatcher dispatcher(g, unlimited, [&](const TriggeredFrame& f, Dispatcher::output_ports_type& op){
		try{
			if (f.flags & WaitKey::SAVE){
				get<0>(op).try_put(f);
			}
			if (f.flags & WaitKey::DISPLAY){
				get<1>(op).try_put(f);
			}
		}
		catch (const exception& e){
			cerr << __FILE__ << "[" << __LINE__ << "] " << e.what() << endl;
			throw;
		}
	});

	/*
	writes triggered frames to disk
	*/
	function_node<TriggeredFrame > writer(g, unlimited, [&](const TriggeredFrame& f) -> continue_msg {
		try{
			path camPath = basePath;
			camPath += std::to_string(f.serial);
			camPath += path::preferred_separator;

			stringstream ss;
			ss << setw(9) << setfill('0') << f.frame_no;

			camPath += ss.str();
			switch (f.frame.channels()){
			case 1:
				camPath += ".pgm";
				break;
			case 3:
				camPath += ".ppm";
				break;
			default:
				throw runtime_error("only 1 or 3 channel images supported");
			}

			imwrite(camPath.string(), f.frame);

			ss.str("");
			ss << "WRITE: " << camPath.string() << endl;
			cerr << ss.str();

			return continue_msg();
		}
		catch (const exception& e){
			cout << __FILE__ << "[" << __LINE__ << "] " << e.what() << endl;
			throw;
		}
	});

	/*
	mapping from camera serial number to output port index
	*/
	map<uint32_t, size_t> cam2op;
	for (int i = 0; i < cams.size(); i++){
		cam2op[cams[i]->serial] = i;
	}

	/*
	normalizes triggered frames for rendering
	*/
	TFHelper<N_CAMS>::Normalizer normalizer(g, unlimited, [&](const TriggeredFrame& f, TFHelper<N_CAMS>::Normalizer::output_ports_type& op){
		try{
			TriggeredFrame fnorm = f;
			fnorm.frame = fnorm.frame.clone();
			Mat ret = fnorm.frame;

			resize(ret.clone(), ret, Size(480, 360));

			switch (ret.channels())
			{
			case 1:
				cvtColor(ret.clone(), ret, CV_GRAY2RGB);
				break;
			case 3:
				//cvtColor(ret.clone(), ret, CV_BGR2GRAY);
				break;
			default:
				throw runtime_error("channels == 1 || channels == 3");
			}

			double minVal, maxVal, alpha, beta;
			minMaxLoc(ret, &minVal, &maxVal);
			alpha = 255. / (maxVal - minVal);
			beta = -255. * minVal / (maxVal - minVal);
			ret.clone().convertTo(ret, CV_8U, alpha, beta);

			ret.copyTo(fnorm.frame);
			TFHelper<N_CAMS>::getOutputPort(cam2op[f.serial], op).try_put(fnorm);
		}
		catch (const exception& e){
			cerr << __FILE__ << "[" << __LINE__ << "] " << e.what() << endl;
			throw;
		}
	});

	/*
	multiplexing of triggered frames with same frame number
	*/
	TFHelper<N_CAMS>::Multiplexer multiplexer = TFHelper<N_CAMS>::getMultiplexer(g);

	/*
	rendering of triggered frames with same frame number
	*/
	function_node<TFHelper<N_CAMS>::TFtuple> renderer(g, unlimited, [&](const TFHelper<N_CAMS>::TFtuple& ftuple) -> continue_msg {
		try{
			vector<Mat> frames = TFHelper<N_CAMS>::getFrames(ftuple);

			while (frames.size() < 4){
				frames.push_back(Mat::zeros(frames[0].size(), frames[0].type()));
			}

			int rows = frames[0].rows, cols = frames[0].cols;
			Mat frame = Mat::zeros(Size(2 * cols, 2 * rows), frames[0].type());
			frames[0].copyTo(frame(Range(0 * rows, 1 * rows), Range(0 * cols, 1 * cols)));
			frames[1].copyTo(frame(Range(0 * rows, 1 * rows), Range(1 * cols, 2 * cols)));
			frames[2].copyTo(frame(Range(1 * rows, 2 * rows), Range(0 * cols, 1 * cols)));
			frames[3].copyTo(frame(Range(1 * rows, 2 * rows), Range(1 * cols, 2 * cols)));

			imshow(winname, frame);

			stringstream ss;
			ss << "RENDER: " << get<0>(ftuple).frame_no << endl;
			cerr << ss.str();

			return continue_msg();
		}
		catch (const exception& e){
			cerr << __FILE__ << "[" << __LINE__ << "] " << e.what() << endl;
			throw;
		}
	});


	make_edge(output_port<0>(dispatcher), writer);
	make_edge(output_port<1>(dispatcher), normalizer);
	TFHelper<N_CAMS>::make_edges(normalizer, multiplexer);
	make_edge(multiplexer, renderer);

	namedWindow(winname);
	uint64_t wkFlags = WaitKey::DISPLAY | WaitKey::SAVE;
	double total_s = double(getTickCount());

	int frame_no;
	double loop_s = 0; // elapsed seconds within loop
	for (frame_no = 0;
		(framecount < 0 || frame_no < framecount) && !((wkFlags ^= waitKey(fps, loop_s)) & WaitKey::QUIT);
		frame_no++) {
		loop_s = double(getTickCount());

		serial_for(size_t(0), cams.size(), [&](size_t i){
			cams[i]->trigger();
		});

		concurrent_vector<TriggeredFrame> frames;

		parallel_for(size_t(0), cams.size(), [&](size_t i){
			try{
				Mat frame = cams[i]->read();
				TriggeredFrame f;
				f.flags = wkFlags;
				f.frame = frame;
				f.frame_no = frame_no;
				f.serial = cams[i]->serial;
				frames.push_back(f);
			}
			catch (const TriggeredCamError& e){
				cerr << e.what() << endl;
			}
		});

		if (frames.size() == cams.size()){
			for_each(frames.begin(), frames.end(), [&](const TriggeredFrame& f){
				dispatcher.try_put(f);
			});
		}

		loop_s = getTickCount() - loop_s;
		loop_s /= getTickFrequency();
		cerr << frame_no << " fps: " << 1. / loop_s << endl;
		//cerr << bitset<sizeof(wkFlags) * 8>(wkFlags) << endl << endl;
	}
	g.wait_for_all();

	total_s = getTickCount() - total_s;
	total_s /= getTickFrequency();
	cout << "avg fps: " << frame_no / total_s << endl;

	return EXIT_SUCCESS;
}

int _tmain(int argc, _TCHAR* argv[]) {
	try {
		return main_graph(argc, argv);
	}
	catch (const exception& e) {
		cerr << e.what() << endl;
		throw;
	}
}
