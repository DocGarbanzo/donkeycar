'''
tub.py
Manage tubs
'''

import os
import json
import tornado.web
import argparse
from stat import ST_ATIME


class TubManager:

    def run(self, args):
        args = self.parse_args(args)
        data_dir = args.data[0]
        WebServer(data_dir).start()

    def parse_args(self, args):
        parser = argparse.ArgumentParser(prog='clean',
                                         usage='%(prog)s [options]')
        parser.add_argument('data', nargs='+', help='paths to data dir '
                                                    'containing tubs')
        parsed_args = parser.parse_args(args)
        return parsed_args


class WebServer(tornado.web.Application):

    def __init__(self, data_path):
        if not os.path.exists(data_path):
            raise ValueError('The path {} does not exist.'.format(data_path))

        this_dir = os.path.dirname(os.path.realpath(__file__))
        static_file_path = os.path.join(this_dir, 'tub_web', 'static')

        handlers = [
            (r"/", tornado.web.RedirectHandler, dict(url="/tubs")),
            (r"/tubs", TubsView, dict(data_path=data_path)),
            (r"/tubs/?(?P<tub_id>[^/]+)?", TubView),
            (r"/api/tubs/?(?P<tub_id>[^/]+)?", TubApi, dict(data_path=data_path)),
            (r"/static/(.*)", tornado.web.StaticFileHandler, {"path": static_file_path}),
            (r"/tub_data/(.*)", tornado.web.StaticFileHandler, {"path": data_path}),
            ]

        settings = {'debug': True}

        super().__init__(handlers, **settings)

    def start(self, port=8886):
        self.port = int(port)
        self.listen(self.port)
        print('Listening on {}...'.format(port))
        tornado.ioloop.IOLoop.instance().start()


class TubsView(tornado.web.RequestHandler):

    def initialize(self, data_path):
        self.data_path = data_path

    def get(self):
        import fnmatch
        dir_list = fnmatch.filter(os.listdir(self.data_path), '*')
        dir_list.sort()
        # Remove '.' files in directory list
        dir_list = [d for d in dir_list if d[0] is not '.']
        data = {"tubs": dir_list}
        self.render("tub_web/tubs.html", **data)


class TubView(tornado.web.RequestHandler):

    def get(self, tub_id):
        data = {'tub': tub_id}
        self.render("tub_web/tub.html", **data)


class TubApi(tornado.web.RequestHandler):

    def initialize(self, data_path):
        self.data_path = data_path

    def image_path(self, tub_path, frame_id):
        return os.path.join(tub_path, str(frame_id) + "_cam-image_array_.jpg")

    def record_path(self, tub_path, frame_id):
        return os.path.join(tub_path, "record_" + frame_id + ".json")

    def clips_of_tub(self, tub_path):
        seqs = [int(f.split("_")[0]) for f in os.listdir(tub_path) if f.endswith('.jpg')]
        seqs.sort()

        entries = ((os.stat(self.image_path(tub_path, seq))[ST_ATIME], seq) for seq in seqs)

        (last_ts, seq) = next(entries)
        clips = [[seq]]
        for next_ts, next_seq in entries:
            clips[-1].append(next_seq)

        return clips

    def get(self, tub_id):
        clips = self.clips_of_tub(os.path.join(self.data_path, tub_id))
        self.set_header("Content-Type", "application/json; charset=UTF-8")
        self.write(json.dumps({'clips': clips}))

    def post(self, tub_id):
        tub_path = os.path.join(self.data_path, tub_id)
        old_clips = self.clips_of_tub(tub_path)
        new_clips = tornado.escape.json_decode(self.request.body)

        import itertools
        old_frames = list(itertools.chain(*old_clips))
        new_frames = list(itertools.chain(*new_clips['clips']))
        frames_to_delete = [str(item) for item in old_frames if item not in new_frames]
        for frm in frames_to_delete:
            os.remove(self.record_path(tub_path, frm))
            os.remove(self.image_path(tub_path, frm))
