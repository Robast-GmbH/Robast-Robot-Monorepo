import 'package:flutter/material.dart';
import 'package:middleware_api_utilities/middleware_api_utilities.dart';
import 'package:web_frontend/constants/web_colors.dart';

class LogPage extends StatefulWidget {
  const LogPage({required this.logName, super.key});
  final String logName;
  @override
  State<LogPage> createState() => _LogPageState();
}

class _LogPageState extends State<LogPage> {
  bool sortAscending = false;
  late Future<String?> loadLog;

  @override
  void initState() {
    super.initState();
    loadLog = MiddlewareApiUtilities().logs.getLog(widget.logName);
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: Text(widget.logName),
        actions: [
          IconButton(
            onPressed: () {
              setState(() {
                sortAscending = !sortAscending;
              });
            },
            icon: const Icon(Icons.swap_vert),
          ),
          const SizedBox(
            width: 16,
          ),
          IconButton(
            onPressed: () {
              setState(() {
                loadLog = MiddlewareApiUtilities().logs.getLog(widget.logName);
              });
            },
            icon: const Icon(Icons.refresh),
          ),
        ],
      ),
      body: FutureBuilder(
        future: loadLog,
        builder: (context, snapshot) {
          if (snapshot.connectionState != ConnectionState.done) {
            return const Center(child: CircularProgressIndicator());
          }
          if (snapshot.data == null) {
            return Center(
              child: Column(
                mainAxisAlignment: MainAxisAlignment.center,
                children: [
                  const Text(
                    'Fehler beim Laden des Logs',
                    style: TextStyle(color: WebColors.secondaryText),
                  ),
                  const SizedBox(height: 16),
                  ElevatedButton(
                    onPressed: () {
                      setState(() {
                        loadLog = MiddlewareApiUtilities().logs.getLog(widget.logName);
                      });
                    },
                    child: const Text('Erneut versuchen'),
                  ),
                ],
              ),
            );
          }

          final logEntries = sortAscending ? snapshot.data!.split('\n') : snapshot.data!.split('\n').reversed.toList();
          if (!sortAscending) {
            logEntries.removeAt(0);
          }
          return ListView.builder(
            itemCount: logEntries.length,
            itemBuilder: (context, index) {
              final logEntry = logEntries[index];
              return Padding(
                padding: const EdgeInsets.all(8),
                child: Text(
                  logEntry,
                  style: TextStyle(
                    color: logEntry.contains('ERROR') ? Colors.redAccent : WebColors.secondaryText,
                  ),
                ),
              );
            },
          );
        },
      ),
    );
  }
}
