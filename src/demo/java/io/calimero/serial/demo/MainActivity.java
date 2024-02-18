/*
    Calimero 2 - A library for KNX network access
    Copyright (c) 2024, 2024 B. Malinowsky

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

    Linking this library statically or dynamically with other modules is
    making a combined work based on this library. Thus, the terms and
    conditions of the GNU General Public License cover the whole
    combination.

    As a special exception, the copyright holders of this library give you
    permission to link this library with independent modules to produce an
    executable, regardless of the license terms of these independent
    modules, and to copy and distribute the resulting executable under terms
    of your choice, provided that you also meet, for each linked independent
    module, the terms and conditions of the license of that module. An
    independent module is a module which is not derived from or based on
    this library. If you modify this library, you may extend this exception
    to your version of the library, but you are not obligated to do so. If
    you do not wish to do so, delete this exception statement from your
    version.
*/

package io.calimero.serial.demo;

import android.os.Bundle;
import android.widget.TextView;

import androidx.appcompat.app.AppCompatActivity;

import java.io.IOException;
import java.io.PrintWriter;
import java.io.StringWriter;
import java.time.Duration;

import io.calimero.serial.provider.jni.TtySerialComProvider;
import io.calimero.serial.spi.SerialCom;
import io.calimero.serial.spi.SerialConnectionProvider;

public class MainActivity extends AppCompatActivity {

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        setContentView(R.layout.activity_main);
        TextView text = findViewById(R.id.textView);

        var provider = new TtySerialComProvider();
        var ports = provider.portIdentifiers();
        text.append("Found serial ports: " + ports + "\n");

        final String portId = "/dev/ttyS0";
        final int baudrate = 9_600;
        final Duration readIntervalTimeout = Duration.ZERO;
        final Duration receiveTimeout = Duration.ofMillis(5);
        final int databits = 8;
        final var stopbits = SerialCom.StopBits.One;
        final var parity = SerialCom.Parity.Even;
        final var flowControl = SerialCom.FlowControl.None;

        record SettingsImpl(String portId, int baudrate, int databits, SerialCom.StopBits stopbits,
                            SerialCom.Parity parity, SerialCom.FlowControl flowControl,
                            Duration readIntervalTimeout, Duration receiveTimeout)
                implements SerialConnectionProvider.Settings {}
        final var settings = new SettingsImpl(portId, baudrate, databits, stopbits, parity,
                flowControl, readIntervalTimeout, receiveTimeout);

        text.append("Opening port " + portId + "\n");
        try (var port = provider.open(settings)) {
            text.append("Opened port " + port + "\n");
//            var tty = (TtySerialCom) port;
//            text.append("Timeouts = " +  tty.getTimeouts() + "\n");
        } catch (IOException e) {
            var trace = new StringWriter();
            e.printStackTrace(new PrintWriter(trace));
            text.append("Error: " + trace + "\n");
        }
    }
}
