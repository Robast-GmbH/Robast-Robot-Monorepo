import 'dart:convert';
import 'dart:io';

import 'package:flutter/material.dart';

class DockerOverview extends StatefulWidget {
  const DockerOverview({super.key});

  @override
  State<DockerOverview> createState() => _DockerOverviewState();
}

class _DockerOverviewState extends State<DockerOverview> {
  final dockerNames = ["backend", "statemachine", "drawer"];
  final isRunning = [false, true, false];
  late Future<List<String>> loadDockerNames;

  Future<List<String>> getDockerContainerNames() async {
    ProcessResult result = await Process.run(
      'docker',
      ['ps', '--format', '{{.Names}}'],
    );

    if (result.exitCode == 0) {
      String output = result.stdout as String;
      List<String> containerNames = LineSplitter.split(output).toList();
      return containerNames;
    } else {
      debugPrint('Error retrieving Docker container names: ${result.stderr}');
      return [];
    }
  }

  Future<List<String>> getDockerLogs(String dockerName) async {
    ProcessResult result = await Process.run(
      'docker',
      ['logs', dockerName],
    );

    if (result.exitCode == 0) {
      String output = result.stdout as String;
      List<String> logData = LineSplitter.split(output).toList();
      return logData;
    } else {
      debugPrint('Error retrieving Docker container names: ${result.stderr}');
      return [];
    }
  }

  @override
  void initState() {
    super.initState();
    loadDockerNames = getDockerContainerNames();
  }

  @override
  Widget build(BuildContext context) {
    return FutureBuilder<List<String>>(
        future: loadDockerNames,
        builder: (context, snapshot) {
          if (!snapshot.hasData) {
            return const Center(
              child: CircularProgressIndicator(),
            );
          }
          return Padding(
            padding: const EdgeInsets.symmetric(
              horizontal: 8,
              vertical: 2,
            ),
            child: ListView(
              children: List.generate(
                snapshot.data!.length,
                (index) => Card(
                  child: ExpansionTile(
                    expandedAlignment: Alignment.topLeft,
                    leading: const Icon(Icons.extension),
                    title: Text(snapshot.data![index]),
                    children: [
                      ConstrainedBox(
                        constraints: const BoxConstraints(maxHeight: 400),
                        child: FutureBuilder<List<String>>(
                          future: getDockerLogs(snapshot.data![index]),
                          builder: (context, logData) {
                            if (!logData.hasData) {
                              return const Center(
                                child: CircularProgressIndicator(),
                              );
                            }
                            return ListView.builder(
                              reverse: true,
                              shrinkWrap: true,
                              itemCount: logData.data!.length,
                              itemBuilder: (context, index) => Padding(
                                padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 8),
                                child: Text(
                                  logData.data![index],
                                  textAlign: TextAlign.start,
                                ),
                              ),
                            );
                          },
                        ),
                      ),
                    ],
                  ),
                ),
              ),
            ),
          );
        });
  }
}
