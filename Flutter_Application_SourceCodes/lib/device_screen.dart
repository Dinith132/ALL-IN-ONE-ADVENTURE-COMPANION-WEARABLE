import 'dart:async';
import 'dart:convert';

import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:flutter_blue_classic/flutter_blue_classic.dart';

class DeviceScreen extends StatefulWidget {
  const DeviceScreen({super.key, required this.connection});

  final BluetoothConnection connection;

  @override
  State<DeviceScreen> createState() => _DeviceScreenState();
}

class _DeviceScreenState extends State<DeviceScreen> {
  TextEditingController _controller = TextEditingController();
  StreamSubscription? _readSubscription;
  final List<Map<String, String>> _messages = [];

  @override
  void initState() {
    _readSubscription = widget.connection.input?.listen((event) {
      if (mounted) {
        setState(() => _messages.add({'sender': 'device', 'message': utf8.decode(event)}));
      }
    });
    super.initState();
  }

  @override
  void dispose() {
    widget.connection.dispose();
    _readSubscription?.cancel();
    super.dispose();
  }

  void _sendMessage(String message) {
    setState(() {
      _messages.add({'sender': 'user', 'message': message});
    });
    try {
      widget.connection.writeString(message);
    } catch (e) {
      if (kDebugMode) print(e);
      ScaffoldMessenger.maybeOf(context)?.showSnackBar(SnackBar(
        content: Text(
          "Error sending to device. Device is ${widget.connection.isConnected ? "connected" : "not connected"}",
        ),
      ));
    }
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: Text(" ${widget.connection.address}"),
      ),
      body: ListView(
        children: [
          Padding(
            padding: const EdgeInsets.symmetric(vertical: 8),
            child: Text(
              "Terminal",
              style: Theme.of(context).textTheme.titleLarge,
            ),
          ),
          Container(
            height: 560.0, // Set the height of the box
            decoration: BoxDecoration(
              border: Border.all(color: Colors.grey),
              borderRadius: BorderRadius.circular(5.0),
            ),
            child: ListView(
              children: [
                for (var message in _messages)
                  Align(
                  alignment: message['sender'] == 'user'
                    ? Alignment.centerRight
                    : Alignment.centerLeft,
                  child: Container(
                    padding: EdgeInsets.symmetric(vertical: 8, horizontal: 12),
                    margin: EdgeInsets.symmetric(vertical: 4),
                    decoration: BoxDecoration(
                    color: message['sender'] == 'user'
                      ? Colors.blue[100]
                      : Colors.grey[300],
                    borderRadius: BorderRadius.circular(8),
                    ),
                    child: Text(
                    message['message']!,
                    style: TextStyle(fontSize: 18), // Increased font size
                    ),
                  ),
                  ),],
            ),
          ),
          TextField(
            controller: _controller,
            decoration: InputDecoration(
              labelText: 'Enter message',
            ),
          ),
          SizedBox(height: 16.0),
          ElevatedButton(
            onPressed: () {
              String message = _controller.text;
              _sendMessage(message);
              _controller.clear();
            },
            child: const Text("Send message"),
          ),
          const Divider(),
        ],
      ),
    );
  }
}