import 'dart:async';
import 'dart:io';

import 'package:sqflite/sqflite.dart';
import 'package:path/path.dart';
import 'package:path_provider/path_provider.dart';

class DatabaseHelper {
  static final DatabaseHelper _instance = DatabaseHelper._internal();
  factory DatabaseHelper() => _instance;
  static Database? _database;
  final StreamController<void> _messageStreamController = StreamController<void>.broadcast();

  Stream<void> get messageStream => _messageStreamController.stream;

  DatabaseHelper._internal();

  Future<Database> get database async {
    if (_database != null) return _database!;
    _database = await _initDatabase();
    return _database!;
  }

  Future<Database> _initDatabase() async {
    String path = join(await getDatabasesPath(), 'messages.db');
    return await openDatabase(
      path,
      version: 1,
      onCreate: (db, version) {
        return db.execute(
        
          'CREATE TABLE messages(id INTEGER PRIMARY KEY, sender TEXT, message TEXT)',
        );
      },
    );
  }

  Future<void> insertMessage(Map<String, String> message) async {
    final db = await database;
    await db.insert('messages', message);
    _messageStreamController.add(null);
  }

  Future<List<Map<String, dynamic>>> getMessages() async {
    final db = await database;
    return await db.query('messages');
  }

  Future<Map<String, dynamic>?> getLastMessage() async {
    final db = await database;
    final List<Map<String, dynamic>> result = await db.query(
      'messages',
      orderBy: 'id DESC',
      limit: 1,
    );
    if (result.isNotEmpty) {
      return result.first;
    } else {
      return null;
    }
  }

  Future<void> clearMessages() async {
    final db = await database;
    await db.delete('messages');
    _messageStreamController.add(null);
  }
}