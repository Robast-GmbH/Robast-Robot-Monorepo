import 'package:flutter/material.dart';

class DockerOverview extends StatefulWidget {
  const DockerOverview({super.key});

  @override
  State<DockerOverview> createState() => _DockerOverviewState();
}

class _DockerOverviewState extends State<DockerOverview> {
  final dockerNames = ["backend", "statemachine", "drawer"];
  final isRunning = [false, true, false];
  @override
  Widget build(BuildContext context) {
    return Padding(
      padding: const EdgeInsets.symmetric(
        horizontal: 8,
        vertical: 2,
      ),
      child: ListView(
        children: List.generate(
          dockerNames.length,
          (index) => Card(
            child: ListTile(
              leading: const Icon(Icons.extension),
              title: Text(dockerNames[index]),
              trailing: IconButton(
                icon: Icon(isRunning[index] ? Icons.pause : Icons.play_arrow),
                onPressed: () {
                  isRunning[index] = !isRunning[index];
                  setState(() {});
                },
              ),
            ),
          ),
        ),
      ),
    );
  }
}
