import 'dart:async';
import 'dart:convert';

import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:flutter_blue_classic/flutter_blue_classic.dart';
import 'package:flutter_blue_classic_example/map_page.dart';

class DeviceScreen extends StatefulWidget {
  const DeviceScreen({super.key, required this.connection});

  final BluetoothConnection connection;

  @override
  State<DeviceScreen> createState() => _DeviceScreenState();
}

class _DeviceScreenState extends State<DeviceScreen> {
  TextEditingController _controller = TextEditingController();
  ScrollController _scrollController = ScrollController();
  StreamSubscription? _readSubscription;
  final List<Map<String, String>> _messages = [];

  @override
  void initState() {
    _readSubscription = widget.connection.input?.listen((event) {
      if (mounted) {
        setState(() {
          _messages.add({'sender': 'device', 'message': utf8.decode(event)});
          _scrollToBottom();
        });
      }
    });
    super.initState();
  }

  @override
  void dispose() {
    widget.connection.dispose();
    _readSubscription?.cancel();
    _scrollController.dispose();
    super.dispose();
  }

  void _sendMessage(String message) {
    setState(() {
      _messages.add({'sender': 'user', 'message': message});
      _scrollToBottom();
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

  void _scrollToBottom() {
    _scrollController.animateTo(
      _scrollController.position.maxScrollExtent,
      duration: Duration(milliseconds: 300),
      curve: Curves.easeOut,
    );
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: Text(" ${widget.connection.address}"),
        backgroundColor: const Color.fromARGB(255, 234, 234, 234),
        actions: [
          IconButton(
            icon: Icon(Icons.map),
                        onPressed: () {
              Navigator.push(
                context,
                MaterialPageRoute(builder: (context) => const MapPage()),
              );
            },
            ),
          IconButton(
            icon: Icon(Icons.settings),
            onPressed: () {},
            
          ),
        ],

      ),
      body: ListView(
        children: [
          Padding(
            padding: const EdgeInsets.symmetric(vertical: 8),
            child: Text(
              "Chats",
              style: Theme.of(context).textTheme.titleLarge,
            ),
          ),
          Container(
            height: 629.0,
            decoration: BoxDecoration(
              border: Border.all(color: Colors.grey),
              borderRadius: BorderRadius.circular(5.0),
            ),
            child: ListView(
              controller: _scrollController,
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
                      child: message['message']!=null&&message['message']!.isNotEmpty
                          ?Text(message['message']!,style:TextStyle(fontSize:18))
                          :SizedBox(),

                    ),
                  ),
              ],
            ),
          ),
          Row(
            children: [
              Expanded(
                child: TextField(
                  controller: _controller,
                  decoration: InputDecoration(
                    labelText: 'Enter message',
                  ),
                ),
              ),
              SizedBox(width: 16.0),
              ElevatedButton(
                onPressed: () {
                  String message = _controller.text;
                  _sendMessage(message);
                  _controller.clear();
                },
                child: const Text("Send"),
              ),
            ],
          ),
          const Divider(),
        ],
      ),
    );
  }
}