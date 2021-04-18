import os
import glob
import logging

from radio_tools import radio_tools
import rospkg
from python_qt_binding import loadUi
from python_qt_binding import QtWidgets, QtCore

logger = logging.getLogger("radio_gui")


def scan_ports(prefix="ttyUSB"):
    path = os.path.join("/dev", prefix + "*")
    ports = glob.glob(path)
    ports.sort()
    logger.debug("Available ports: %s", ports)
    return ports


def enable_all_buttons_in_group(group, enabled):
    buttons = group.findChildren(QtWidgets.QPushButton)
    for button in buttons:
        button.setEnabled(enabled)


def show_error_box(message):
    error_diag = QtWidgets.QErrorMessage()
    error_diag.showMessage("{}".format(message))
    error_diag.exec_()


def show_message_box(message):
    message_diag = QtWidgets.QMessageBox()
    message_diag.setText("{}".format(message))
    message_diag.exec_()


class ProgressDialog(QtWidgets.QDialog):
    def __init__(self, parent=None):
        super(ProgressDialog, self).__init__(parent)
        self.progressbar = QtWidgets.QProgressBar(self)
        self.progressbar.setMinimum(0)
        self.progressbar.setMaximum(0)
        self.progressbar.setFormat("%v/%m")

    @QtCore.pyqtSlot(int, int)
    def update_progress(self, progress, maximum):
        self.progressbar.setMaximum(maximum)
        self.progressbar.setValue(progress)


class ComboBox(QtWidgets.QComboBox):
    popup = QtCore.pyqtSignal()

    def showPopup(self):
        self.popup.emit()
        super(ComboBox, self).showPopup()


class QTextEditLogger(logging.Handler):
    def __init__(self, textedit_widget):
        super(QTextEditLogger, self).__init__()
        self.widget = textedit_widget
        self.widget.setReadOnly(True)

    def emit(self, record):
        msg = self.format(record)
        self.widget.append(msg)


class Worker(QtCore.QObject):
    ret = QtCore.pyqtSignal(object)
    finished = QtCore.pyqtSignal()

    def __init__(self, fun, *args, **kwargs):
        super(Worker, self).__init__()
        self.fun = fun
        self.args = args
        self.kwargs = kwargs

    def run(self):
        self.ret.emit(self.fun(*self.args, **self.kwargs))
        self.finished.emit()


class UploadWorker(QtCore.QObject):
    ret = QtCore.pyqtSignal(object)
    finished = QtCore.pyqtSignal()
    program_progress = QtCore.pyqtSignal(int, int)
    verify_progress = QtCore.pyqtSignal(int, int)

    def __init__(self, fun=None, path=None):
        super(UploadWorker, self).__init__()
        self.upload = fun
        self.path = path

    def set_fun(self, fun):
        self.upload = fun

    def set_path(self, path):
        self.path = path

    def on_program_progress(self, progress, total):
        self.program_progress.emit(progress, total)

    def on_verify_progress(self, progress, total):
        self.verify_progress.emit(progress, total)

    def run(self):
        ret = None
        try:
            ret = self.upload(self.path)
        except Exception as e:
            logger.error("Error during firmware upload: %s", e)
        self.ret.emit(ret)
        self.finished.emit()


class ReadWriteParamsWorker(QtCore.QObject):
    ret = QtCore.pyqtSignal(object)
    finished = QtCore.pyqtSignal()
    progress = QtCore.pyqtSignal(int, int)

    def __init__(self, fun, param_names=None, param_values=None):
        super(ReadWriteParamsWorker, self).__init__()
        self.names = param_names
        self.values = param_values
        self.fun = fun

    def set_params(self, names, values=None):
        self.names = names
        self.values = values

    def run(self):
        ret_values = dict()
        if self.names is not None:
            self.progress.emit(0, len(self.names))
            for i, param in enumerate(self.names):
                if self.values is None:
                    ret_values[param] = self.fun(param)
                else:
                    ret_values[param] = self.fun(param, self.values[i])
                self.progress.emit(i + 1, len(self.names))
        self.ret.emit(ret_values)
        self.finished.emit()


class RadioConfiguratorWidget(QtWidgets.QWidget):
    BAUDS = [2400, 4800, 9600, 19200, 38400, 57600, 115200]
    BAUD_DEFAULT = 57600

    connected = QtCore.pyqtSignal()
    disconnected = QtCore.pyqtSignal()
    at_entered = QtCore.pyqtSignal()
    at_exited = QtCore.pyqtSignal()

    def __init__(self):
        super(RadioConfiguratorWidget, self).__init__()
        self.configurator = radio_tools.Configurator()
        self._is_connected = False
        self._in_at_mode = False

        self._enter_at_thread = None
        self._enter_at_worker = None

        self._read_params_thread = None
        self._read_params_worker = None
        self.setup_background_threads()
        ui_file = os.path.join(rospkg.RosPack().get_path("radio_tools"),
                               "resource", "radio_configurator.ui")

        loadUi(ui_file, self)
        self.at_entered.connect(self.on_at_entered)
        self.at_exited.connect(self.on_at_exited)
        self.logging_handler = QTextEditLogger(self.console_text)
        self.logging_handler.setFormatter(
            logging.Formatter("%(asctime)s - %(levelname)s - %(message)s"))
        self.logger = logging.getLogger("radio_gui")
        self.logger.addHandler(self.logging_handler)
        self.logger.setLevel(logging.DEBUG)
        self.init_logging_radiobuttons()
        self.connected.connect(self.on_connected)
        self.disconnected.connect(self.on_disconnected)
        self.init_connection_group()
        self.eeprom_group.setEnabled(False)
        self.load_settings_progressbar.reset()
        self.init_at_group()
        self.init_param_widgets()

    def init_logging_radiobuttons(self):
        def connect_button_toggled(button):
            button.toggled.connect(
                lambda: self.handle_logging_radiobutton(button))

        buttons = self.logger_level_group.findChildren(QtWidgets.QRadioButton)
        for button in buttons:
            connect_button_toggled(button)
        self.info_radiobutton.setChecked(True)

    def handle_logging_radiobutton(self, button):
        if not button.isChecked():
            return
        if button.text() == "Debug":
            self.logger.setLevel(logging.DEBUG)
            self.logger.debug("Setting debug level to %s.",
                              button.text().upper())
        elif button.text() == "Info":
            self.logger.debug("Setting debug level to %s.",
                              button.text().upper())
            self.logger.setLevel(logging.INFO)
        elif button.text() == "Warn":
            self.logger.debug("Setting debug level to %s.",
                              button.text().upper())
            self.logger.setLevel(logging.WARN)
        elif button.text() == "Error":
            self.logger.debug("Setting debug level to %s.",
                              button.text().upper())
            self.logger.setLevel(logging.ERROR)
        else:
            self.logger.error("Unhandled logging level: %s.", button.text())

    def setup_background_threads(self):
        def connect_worker_and_thread(worker, thread, return_cb, finished_cb):
            worker.moveToThread(thread)
            thread.started.connect(worker.run)
            worker.ret.connect(return_cb)
            worker.finished.connect(finished_cb)

        self._enter_at_thread = QtCore.QThread()
        self._enter_at_worker = Worker(self.configurator.enter_at_mode)
        connect_worker_and_thread(worker=self._enter_at_worker,
                                  thread=self._enter_at_thread,
                                  return_cb=self.on_enter_at_result,
                                  finished_cb=self.on_enter_at_finished)

        self._read_params_thread = QtCore.QThread()
        self._read_params_worker = ReadWriteParamsWorker(
            self.configurator.read_param)
        connect_worker_and_thread(worker=self._read_params_worker,
                                  thread=self._read_params_thread,
                                  return_cb=self.on_read_params_result,
                                  finished_cb=self.on_read_params_finished)
        self._read_params_worker.progress.connect(self.update_load_progressbar)

        self._write_params_thread = QtCore.QThread()
        self._write_params_worker = ReadWriteParamsWorker(
            self.configurator.write_param)
        connect_worker_and_thread(worker=self._write_params_worker,
                                  thread=self._write_params_thread,
                                  return_cb=self.on_write_params_result,
                                  finished_cb=self.on_write_params_finished)
        self._write_params_worker.progress.connect(
            self.update_copy_settings_progressbar)

        self._upload_firmware_thread = QtCore.QThread()
        self._upload_firmware_worker = UploadWorker()
        connect_worker_and_thread(worker=self._upload_firmware_worker,
                                  thread=self._upload_firmware_thread,
                                  return_cb=self.on_upload_firmware_result,
                                  finished_cb=self.on_upload_firmware_finished)
        self._upload_firmware_worker.set_fun(
            lambda path: self.configurator.upload_firmware(
                path, self._upload_firmware_worker.on_program_progress, self.
                _upload_firmware_worker.on_verify_progress))
        self._upload_firmware_worker.program_progress.connect(
            self.on_program_progress)
        self._upload_firmware_worker.verify_progress.connect(
            self.on_verify_progress)

    def on_enter_at_finished(self):
        self._enter_at_thread.quit()
        self._enter_at_thread.wait()

    def on_read_params_finished(self):
        self._read_params_thread.quit()
        self._read_params_thread.wait()

    def on_write_params_finished(self):
        self._write_params_thread.quit()
        self._write_params_thread.wait()

    def on_upload_firmware_finished(self):
        self._upload_firmware_thread.quit()
        self._upload_firmware_thread.wait()
        self.upload_button.setEnabled(True)

    @QtCore.pyqtSlot(object)
    def on_upload_firmware_result(self, result):
        if not result:
            txt = "Failed to upload/verify firmware!"
            self.logger.error(txt)
            show_error_box(txt)
        else:
            txt = "Firmware upload completed successfully."
            self.logger.info(txt)
            show_message_box(txt)

    def init_connection_group(self):
        self.port_combobox.popup.connect(self.update_port_combobox)
        self.baud_combobox.addItems([str(baud) for baud in self.BAUDS])
        for i in range(len(self.BAUDS)):
            if str(self.BAUD_DEFAULT) == self.baud_combobox.itemText(i):
                self.baud_combobox.setCurrentIndex(i)

    def init_at_group(self):
        self.at_group.setEnabled(False)
        enable_all_buttons_in_group(self.at_group, False)
        self.enter_at_button.setEnabled(True)

    def update_port_combobox(self):
        ports = scan_ports()
        current_text = self.port_combobox.currentText()
        self.port_combobox.clear()
        self.port_combobox.addItems(ports)
        if current_text in ports:
            for i in range(len(ports)):
                if current_text == self.port_combobox.itemText(i):
                    self.port_combobox.setCurrentIndex(i)
                    break

    @QtCore.pyqtSlot(int, int)
    def on_program_progress(self, progress, total):
        self.program_progressbar.setMinimum(0)
        self.program_progressbar.setMaximum(total)
        self.program_progressbar.setValue(progress)

    @QtCore.pyqtSlot(int, int)
    def on_verify_progress(self, progress, total):
        self.verify_progressbar.setMinimum(0)
        self.verify_progressbar.setMaximum(total)
        self.verify_progressbar.setValue(progress)

    @QtCore.pyqtSlot()
    def on_connected(self):
        self.logger.info("Connected to '%s@%s'", self.configurator.port.port,
                         self.configurator.port.baudrate)
        self.connect_button.setEnabled(False)
        self.disconnect_button.setEnabled(True)

        self.eeprom_group.setEnabled(True)
        self.firmware_group.setEnabled(True)
        self.load_settings_button.setEnabled(False)
        self.copy_settings_button.setEnabled(False)
        self.at_group.setEnabled(True)
        self.connection_status_label.setText("connected")
        self.do_enter_at_mode()

    @QtCore.pyqtSlot()
    def on_connect_button_clicked(self):
        port = self.port_combobox.currentText()
        baud = int(self.baud_combobox.currentText())
        self.configurator.port.port = port
        self.configurator.port.baudrate = baud
        try:
            if not self.configurator.port.is_open:
                self.configurator.port.open()
        except Exception as e:
            show_error_box(e)
            self.eeprom_group.setEnabled(False)
            self.connection_status_label.setText("failed to connect")
        else:
            self.connected.emit()

    @QtCore.pyqtSlot()
    def on_disconnected(self):
        self.disconnect_button.setEnabled(False)
        self.connect_button.setEnabled(True)
        self.connection_status_label.setText("disconnected")
        self.at_group.setEnabled(False)
        self.eeprom_group.setEnabled(False)
        self.firmware_group.setEnabled(False)
        self.load_settings_button.setEnabled(False)
        self.copy_settings_button.setEnabled(False)

    @QtCore.pyqtSlot()
    def on_disconnect_button_clicked(self):
        self.do_disconnect()

    def do_disconnect(self):
        self.configurator.exit_at_mode()
        self.configurator.port.flush()
        self.configurator.port.close()
        self.disconnected.emit()

    def do_enter_at_mode(self):
        if self._enter_at_thread.isRunning():
            show_error_box("You are already trying to enter AT mode.")
        else:
            self.enter_at_button.setEnabled(False)
            self.logger.info("Entering AT mode...")
            self.at_mode_progressbar_connecting()
            self._enter_at_thread.start()

    @QtCore.pyqtSlot()
    def on_enter_at_button_clicked(self):
        self.do_enter_at_mode()

    @QtCore.pyqtSlot()
    def on_at_entered(self):
        self.at_mode_progressbar_connected()
        enable_all_buttons_in_group(self.at_group, True)
        self.enter_at_button.setEnabled(False)
        enable_all_buttons_in_group(self.eeprom_group, True)

    @QtCore.pyqtSlot()
    def on_at_exited(self):
        enable_all_buttons_in_group(self.at_group, False)
        enable_all_buttons_in_group(self.eeprom_group, False)
        self.enter_at_button.setEnabled(True)
        self.at_mode_progressbar_disconnected()

    @QtCore.pyqtSlot(object)
    def on_enter_at_result(self, success):
        if success:
            self.logger.info("Entered AT mode successfully.")
            self.at_entered.emit()
            self.start_read_all_params_thread()
        else:
            self.logger.error("Failed to enter AT mode.")
            self.at_mode_progressbar_disconnected()
            self.enter_at_button.setEnabled(True)
            show_error_box(
                "Failed to enter AT mode. Make sure you are connected "
                "to the correct device and selected the appropriate baud rate.")

    @QtCore.pyqtSlot()
    def on_exit_at_button_clicked(self):
        self.configurator.exit_at_mode()
        self.logger.info("Exited AT mode.")
        self.at_exited.emit()

    @QtCore.pyqtSlot()
    def on_reboot_button_clicked(self):
        self.configurator.reboot()
        self.logger.info("Rebooted.")
        enable_all_buttons_in_group(self.at_group, False)
        enable_all_buttons_in_group(self.eeprom_group, False)
        self.enter_at_button.setEnabled(True)
        self.at_mode_progressbar_disconnected()

    @QtCore.pyqtSlot()
    def on_browse_button_clicked(self):
        path = QtWidgets.QFileDialog.getOpenFileName(self, "Select Firmware", filter="Intel Hex file (*.ihx)")[0]
        self.logger.debug("Firmware selected: %s", path)
        if os.path.isfile(path):
            self.file_lineedit.setText(path)

    @QtCore.pyqtSlot()
    def on_upload_button_clicked(self):
        path = self.file_lineedit.text()
        if os.path.isfile(path):
            self.start_upload_firmware_thread(path)
        else:
            show_error_box("The specified firmware file '{}' does not exist.".format(path))

    @QtCore.pyqtSlot()
    def on_load_settings_button_clicked(self):
        self.logger.debug("Load settings button clicked.")
        self.start_read_all_params_thread()

    def start_read_all_params_thread(self):
        if self._read_params_thread.isRunning():
            self.logger.warn("Trying to read EEPROM, but the responsible "
                             "thread is already running.")
            show_error_box("Already reading EEPROM...")
        else:
            self.logger.info("Refreshing current EEPROM parameters.")
            params = self.configurator.EEPROM_PARAMETERS.keys()
            self.load_settings_progressbar.setMinimum(0)
            self.load_settings_progressbar.setMaximum(0)
            self._read_params_worker.set_params(params)
            self._read_params_thread.start()

    def start_write_params_thread(self):
        if self._write_params_thread.isRunning():
            show_error_box("Already copying parameters to radio.")
        else:
            params = self.get_current_params()
            names = list(params.keys())
            values = [params[name] for name in names]
            self._write_params_worker.set_params(names, values)
            self._write_params_thread.start()

    def start_upload_firmware_thread(self, path):
        if self._upload_firmware_thread.isRunning():
            show_error_box("Already uploading firmware...")
        else:
            self.upload_button.setEnabled(False)
            self.at_exited.emit()
            self._upload_firmware_worker.set_path(path)
            self._upload_firmware_thread.start()

    def update_load_progressbar(self, progress, maximum):
        self.load_settings_progressbar.setMaximum(maximum)
        self.load_settings_progressbar.setValue(progress)

    def update_copy_settings_progressbar(self, progress, maximum):
        self.copy_settings_progressbar.setMaximum(maximum)
        self.copy_settings_progressbar.setValue(progress)

    @QtCore.pyqtSlot(object)
    def on_read_params_result(self, params):
        if params is None:
            self.logger.error("Failed to read EEPROM parameters!")
            show_error_box("Failed to read EEPROM!")
        else:
            self.logger.debug("Received params: %s", params)
            writable_params = dict()
            for name in params:
                if radio_tools.Configurator.EEPROM_PARAMETERS[name]["readonly"]:
                    continue
                writable_params[name] = params[name]
                self.set_param_combobox(name, params[name])
            self.radio_params = writable_params

    @QtCore.pyqtSlot(object)
    def on_write_params_result(self, result):
        if not all(result):
            show_error_box("Some parameters could not be written.")

    @QtCore.pyqtSlot()
    def on_set_default_button_clicked(self):
        self.logger.debug("Set default button clicked.")
        col = self.get_eeprom_default_column()
        self.copy_column_to_desired(col)

    def at_mode_progressbar_connecting(self):
        self.at_mode_progressbar.setMaximum(0)
        self.at_mode_progressbar.setMinimum(0)
        self.at_mode_progressbar.setEnabled(True)

    def at_mode_progressbar_connected(self):
        self.at_mode_progressbar.setMaximum(1)
        self.at_mode_progressbar.setMinimum(0)
        self.at_mode_progressbar.setValue(1)
        self.at_mode_progressbar.setEnabled(True)

    def at_mode_progressbar_disconnected(self):
        self.at_mode_progressbar.setMaximum(1)
        self.at_mode_progressbar.setMinimum(0)
        self.at_mode_progressbar.setValue(1)
        self.at_mode_progressbar.setEnabled(False)

    @QtCore.pyqtSlot()
    def on_get_standard_settings_button_clicked(self):
        self.set_standard_settings()

    @QtCore.pyqtSlot()
    def on_compare_button_clicked(self):
        self.show_compare_dialog()

    @QtCore.pyqtSlot()
    def on_copy_settings_button_clicked(self):
        if self.show_confirm_changes_dialog():
            if self.show_eeprom_warning():
                self.start_write_params_thread()

    def set_standard_settings(self):
        params = radio_tools.Configurator.get_default_params()
        for name in params:
            self.set_default_combobox(name)

    def get_combobox_ref(self, name):
        try:
            combobox = getattr(self, name)
        except AttributeError:
            self.logger.warn("No widget with name: '%s' was found.", name)
            return None
        return combobox

    def set_param_combobox(self, param_name, value):
        name = param_name + "_combobox"
        combobox = self.get_combobox_ref(name)
        if combobox is None:
            return False
        index = combobox.findText(str(value))
        if index < 0:
            return False
        else:
            combobox.setCurrentIndex(index)
            return True

    def set_default_combobox(self, param_name):
        try:
            value = radio_tools.Configurator.get_default_params()[param_name]
        except KeyError:
            self.logger.error("Could not find default value for '%s'.",
                              param_name)
            return False
        return self.set_param_combobox(param_name, value)

    def get_param_combobox(self, param_name):
        name = param_name + "_combobox"
        combobox = self.get_combobox_ref(name)
        if combobox is None:
            return None
        return combobox.currentText()

    def init_param_combobox(self, param_name, options, default):
        options = [str(option) for option in options]
        name = param_name + "_combobox"
        combobox = self.get_combobox_ref(name)
        if combobox is None:
            return False
        combobox.clear()
        combobox.addItems(options)
        if default is None:
            return True
        index = combobox.findText(str(default))
        if index < 0:
            return False
        else:
            combobox.setCurrentIndex(index)
            return True

    def init_param_widgets(self, set_default=False):
        params = radio_tools.Configurator.EEPROM_PARAMETERS
        for name in params:
            if params[name]["readonly"]:
                continue
            options = params[name]["options"]
            default = params[name]["default"] if set_default else None
            self.init_param_combobox(name, options, default)

    def get_current_params(self):
        names = list(radio_tools.Configurator.get_default_params().keys())
        params = dict()
        for name in names:
            params[name] = self.get_param_combobox(name)
        return params

    def get_changed_params(self):
        current_params = self.get_current_params()
        diff = dict()
        for name in current_params:
            old = int(self.radio_params[name])
            new = int(current_params[name])
            if old != new:
                diff[name] = dict(old=old, new=new)
        return diff

    def show_compare_dialog(self):
        diff = self.get_changed_params()
        if diff:
            text = "Following parameters changed:\n\n"
            for name in diff:
                text += "{}: {} -> {}\n".format(name, diff[name]["old"],
                                                diff[name]["new"])
        else:
            text = "No paramters changed."
        show_message_box(text)

    def show_confirm_changes_dialog(self):
        diff = self.get_changed_params()
        if diff:
            text = "Following parameters will be changed:\n\n"
            for name in diff:
                text += "{}: {} -> {}\n".format(name, diff[name]["old"],
                                                diff[name]["new"])
        else:
            text = "No paramters changed."
        text += "\n\n Continue?"
        diag = QtWidgets.QMessageBox
        reply = diag.question(self, "Confirm changes", text, diag.Yes | diag.No)
        if reply == diag.Yes:
            return True
        else:
            return False

    def show_eeprom_warning(self):
        text = (
            "Parameters are going to be written to the EEPROM.\n\nKeep in "
            "mind that the number of EEPROM writes is limited and this action "
            "should be only performed if necessary.\n\nContinue?")
        diag = QtWidgets.QMessageBox
        reply = diag.warning(self, "Confirm EEPROM write", text,
                             diag.Yes | diag.No)
        if reply == diag.Yes:
            return True
        else:
            return False

    def closeEvent(self, event):
        self.do_disconnect()
        event.accept()
