import 'package:flutter/material.dart';
import 'database_helper.dart';

class DatabasePage extends StatelessWidget {
  const DatabasePage({Key? key}) : super(key: key);

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: const Text('Database'),
      ),
      body: StreamBuilder<void>(
        stream: DatabaseHelper().messageStream,
        builder: (context, snapshot) {
          return FutureBuilder<List<Map<String, dynamic>>>(
            future: DatabaseHelper().getMessages(),
            builder: (context, snapshot) {
              if (snapshot.connectionState == ConnectionState.waiting) {
                return const Center(child: CircularProgressIndicator());
              } else if (snapshot.hasError) {
                return Center(child: Text('Error: ${snapshot.error}'));
              } else if (!snapshot.hasData || snapshot.data!.isEmpty) {
                return const Center(child: Text('No messages found.'));
              } else {
                return ListView.builder(
                  itemCount: snapshot.data!.length,
                  itemBuilder: (context, index) {
                    final message = snapshot.data![index];
                    return ListTile(
                      title: Text(message['message'] ?? 'No message'),
                      subtitle: Text('Sender: ${message['sender']}'),
                    );
                  },
                );
              }
            },
          );
        },
      ),
    );
  }
}