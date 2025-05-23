import 'dart:async';
import 'dart:convert';

import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:flutter_blue_classic/flutter_blue_classic.dart';
import 'package:flutter_blue_classic_example/database_page.dart';
import 'package:flutter_blue_classic_example/map_page.dart';
import 'package:flutter_blue_classic_example/medical_records_page.dart';
import 'database_helper.dart';

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
  List<Map<String, dynamic>> _messages = [];
  Map<String ,dynamic> _data_last={};
  Map<String, dynamic> _msg = {};
  String out_msg=""; // New map variable to store 'our_location'
  Map<String , dynamic> temp={};


  @override
  void initState() {
    super.initState();
    _loadMessages();
    _readSubscription = widget.connection.input?.listen((event) {
      if (mounted) {
        setState(() {
          _storeMessage({'sender': 'device', 'message': utf8.decode(event)});
          _scrollToBottom();
        });
      }
    });
  }

  Future<void> _loadMessages() async {
    _messages = await DatabaseHelper().getMessages();
    setState(() {});
  }

  Future<void> _storeMessage(Map<String, String> message) async {
    await DatabaseHelper().insertMessage(message);
    _messages = await DatabaseHelper().getMessages();
    setState(() {});
  }

  Future<void> _clearMessages() async {
    await DatabaseHelper().clearMessages();
    _messages = [];
    setState(() {});
  }

    Future<void> _loaddata(String m) async {
      // Map<String , dynamic> temp=json.decode(m);
        try {
          Map<String, dynamic> temp = json.decode(m);
          // Process the decoded JSON data
        } catch (e) {
          if (kDebugMode) {
            print('Error decoding JSON: $e');
          }
          // Skip the message if it is not in JSON format
        }


    // var lastMessage = await DatabaseHelper().getLastMessage();
    // if (lastMessage == null) return;

    // if (mounted ) {
    //   final_msg=lastMessage;
    //     setState(() {
    //       _data_last = lastMessage;
    //       if (_data_last.containsKey('message')) {
    //         _msg = json.decode(_data_last['message']); // Decode JSON string
    //         if (_msg.containsKey('message')) {
    //           out_msg = _msg['message']['text'];
    //           print('Loaded location: $out_msg');
    //          // Debug print
    //         } else {
    //           print('No our_location data found in the message'); // Debug print
    //         }

    //     }
    //   });
    // }
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
      _storeMessage({'sender': 'user', 'message': message});
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
            icon: Icon(Icons.medical_information),
                        onPressed: () {
              Navigator.push(
                context,
                MaterialPageRoute(builder: (context) => const MedicalRecordsPage()),
              );
            },
            ),
          IconButton(
            icon: Icon(Icons.more),
            onPressed: () {
              Navigator.push(
                context,
                MaterialPageRoute(builder: (context) => const DatabasePage()),
              );
            },
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
                      child: () {
                        if (message['message'] != null && message['message']!.isNotEmpty) {

                                  try {
                                      Map<String, dynamic> temp = jsonDecode(message['message']!);
                                      print(temp['message']['text']);

                                      return Text(temp['message']['text'], style: TextStyle(fontSize: 18));
                                      // Process the decoded JSON data
                                    } catch (e) {
                                      if (kDebugMode) {
                                        print('Error decoding JSON: $e');
                                      }
                                      // return Text("=========================", style: TextStyle(fontSize: 18));
                                      // Skip the message if it is not in JSON format
                                    }

                          
                        } else {
                          return SizedBox();
                        }
                      }(),
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
              SizedBox(width: 16.0),
              ElevatedButton(
                onPressed: _clearMessages,
                child: const Text("Clear"),
              ),
            ],
          ),
          const Divider(),
        ],
      ),
    );
  }
}