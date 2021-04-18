from qt_gui.plugin import Plugin
from radio_tools.radio_gui import RadioConfiguratorWidget


class RadioConfiguratorPlugin(Plugin):
    def __init__(self, context):
        super(RadioConfiguratorPlugin, self).__init__(context)
        self.setObjectName("RadioConfiguratorPlugin")

        context.add_widget(RadioConfiguratorWidget())

    def shutdown_plugin(self):
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass
