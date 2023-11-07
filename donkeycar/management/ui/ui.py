import os
from kivy.logger import Logger, LOG_LEVELS
from kivy.clock import Clock
from kivy.app import App
from kivy.uix.boxlayout import BoxLayout
from kivy.properties import ObjectProperty
from kivy.lang.builder import Builder
from kivy.core.window import Window
from kivy.uix.screenmanager import Screen

# from donkeycar.management.ui.car_screen import CarScreen
# from donkeycar.management.ui.pilot_screen import PilotScreen
from donkeycar.management.ui.train_screen import TrainScreen
from donkeycar.management.ui.tub_screen import TubScreen
# from donkeycar.management.ui.layout_screen import LayoutScreen
from common import *

Logger.setLevel(LOG_LEVELS["info"])


Window.size = (800, 1000)


class Header(BoxLayout):
    title = StringProperty()
    description = StringProperty()


class ScreenSelector(BackgroundBoxLayout):
    def disable_only(self, bar_name):
        this_button_name = bar_name + '_btn'
        for button_name, button in self.ids.items():
            button.disabled = button_name == this_button_name

    def enable_disable_all(self, enable=True):
        for button_name, button in self.ids.items():
            button.disabled = not enable


class StartScreen(Screen):
    img_path = os.path.realpath(os.path.join(
        os.path.dirname(__file__),
        '../../parts/web_controller/templates/'
        'static/donkeycar-logo-sideways.png'))
    pass


class DonkeyApp(App):
    title = 'Donkey Manager'

    def after_init(self, obj):
        self.root.ids.tub_screen.ids.tub_loader.update_tub()
        #self.root.ids.start_screen.ids.tab_bar.enable_disable_all(True)
        #self.root.ids.start_screen.ids.status.text = 'Donkey ready'

    def initialise(self, event):
        # self.root.ids.start_screen.ids.tab_bar.enable_disable_all(False)
        self.root.ids.tub_screen.ids.config_manager.load_action()
        # self.root.ids.pilot_screen.initialise(event)
        #self.root.ids.car_screen.initialise()
        # This builds the graph which can only happen after everything else
        # has run, therefore delay until the next round.
        Clock.schedule_once(self.after_init)

    def build(self):
        # the builder returns the screen manager
        dm = Builder.load_file(os.path.join(os.path.dirname(__file__), 'ui.kv'))
        Window.bind(on_request_close=self.on_request_close)
        Clock.schedule_once(self.initialise)
        return dm

    def on_request_close(self, *args):
        tub = self.root.ids.tub_screen.ids.tub_loader.tub
        if tub:
            tub.close()
        Logger.info("Good bye Donkey")
        return False


def main():
    DonkeyApp().run()


if __name__ == '__main__':
    main()
