import 'dart:convert';

import 'package:flutter/material.dart';
import 'package:flutter_blue_classic_example/database_helper.dart';

class MedicalRecordsPage extends StatefulWidget {
  const MedicalRecordsPage({Key? key}) : super(key: key);

  @override
  _MedicalRecordsPageState createState() => _MedicalRecordsPageState();
}

class _MedicalRecordsPageState extends State<MedicalRecordsPage> {
  Map<String, dynamic> _messages = {};
  Map<String, dynamic> _msg = {}; // New map variable to store 'our_location'
  Map<String, dynamic> _ourMed = {}; // Variable to store 'our_location'

  @override
  void initState() {
    super.initState();
    _loaddata();
  }



  Future<void> _loaddata() async {
    var lastMessage = await DatabaseHelper().getLastMessage();
    if (lastMessage == null) return;

    if (mounted) {
      setState(() {
        _messages = lastMessage;
        if (_messages.containsKey('message')) {
          _msg = json.decode(_messages['message']); // Decode JSON string
          if (_msg.containsKey('heart rate')) {
            _ourMed = _msg['heart rate'];
            print('Loaded Med: $_ourMed'); // Debug print
          } else {
            print('No Heart rate data found in the message'); // Debug print
          }
        } else {
          print(_messages.keys); // Debug print
        }
      });
    }
  }


  @override
  void dispose() {
    super.dispose();
    // Cancel any async task here
  }


  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: const Text('Medical Records'),
      ),
      body: Center(
        child: const Text('Medical Records Content Here'),
      ),
    );
  }
}