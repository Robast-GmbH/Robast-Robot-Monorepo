import 'package:flutter/material.dart';
import 'package:middleware_api_utilities/middleware_api_utilities.dart';
import 'package:web_frontend/constants/web_colors.dart';
import 'package:web_frontend/pages/log_page.dart';

class LogsPage extends StatefulWidget {
  const LogsPage({super.key});

  @override
  State<LogsPage> createState() => _LogsPageState();
}

class _LogsPageState extends State<LogsPage> {
  late Future<List<String>?> loadAvailableLogs;

  @override
  void initState() {
    loadAvailableLogs = MiddlewareApiUtilities().logs.getAvailableLogs();
    super.initState();
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: const Text('Logs'),
      ),
      body: FutureBuilder(
        future: loadAvailableLogs,
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
                    'Fehler beim Laden der Logs',
                    style: TextStyle(color: WebColors.secondaryText),
                  ),
                  const SizedBox(height: 16),
                  ElevatedButton(
                    onPressed: () {
                      setState(() {
                        loadAvailableLogs = MiddlewareApiUtilities().logs.getAvailableLogs();
                      });
                    },
                    child: const Text('Erneut versuchen'),
                  ),
                ],
              ),
            );
          }
          return ListView(
            children: snapshot.data!
                .map(
                  (logName) => ListTile(
                    contentPadding: const EdgeInsets.only(left: 32),
                    title: Text(logName),
                    onTap: () {
                      Navigator.push(
                        context,
                        MaterialPageRoute<String>(
                          builder: (context) => LogPage(
                            logName: logName,
                          ),
                        ),
                      );
                    },
                  ),
                )
                .toList(),
          );
        },
      ),
    );
  }
}
