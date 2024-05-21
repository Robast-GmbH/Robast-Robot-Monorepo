import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:rmf_api/rmf_api.dart';
import 'package:web_frontend/models/controller/task_creation_controller.dart';
import 'package:web_frontend/models/provider/rmf_provider.dart';

class PatrolTaskCreationView extends StatefulWidget {
  const PatrolTaskCreationView({required this.controller, super.key});

  final TaskCreationController controller;

  @override
  State<PatrolTaskCreationView> createState() => _PatrolTaskCreationViewState();
}

class _PatrolTaskCreationViewState extends State<PatrolTaskCreationView> {
  @override
  Widget build(BuildContext context) {
    return Column(
      children: [
        Padding(
          padding: const EdgeInsets.symmetric(horizontal: 16, vertical: 8),
          child: TextField(
            keyboardType: TextInputType.number,
            decoration: const InputDecoration(
              border: OutlineInputBorder(),
              labelText: 'Rounds',
            ),
            onChanged: (value) {
              final newValue = int.tryParse(value);
              if (newValue != null) {
                widget.controller.rounds = newValue;
              }
            },
          ),
        ),
        Expanded(
          child: ListView.builder(
            itemCount: widget.controller.places.length + 1,
            itemBuilder: (context, index) {
              if (index < widget.controller.places.length) {
                return _buildPatrolRouteDropdown(index);
              } else {
                return Padding(
                  padding: const EdgeInsets.symmetric(vertical: 16),
                  child: Center(
                    child: Container(
                      decoration: const BoxDecoration(
                        color: Colors.blue,
                        shape: BoxShape.circle,
                      ),
                      child: IconButton(
                        icon: const Icon(Icons.add),
                        onPressed: () {
                          widget.controller.places.add('');
                          setState(() {});
                        },
                      ),
                    ),
                  ),
                );
              }
            },
          ),
        ),
      ],
    );
  }

  Dismissible _buildPatrolRouteDropdown(int index) {
    String? selectedNode;
    if (widget.controller.places[index].isNotEmpty) {
      selectedNode = widget.controller.places[index];
    }

    return Dismissible(
      key: UniqueKey(),
      direction: DismissDirection.endToStart,
      background: Container(
        alignment: Alignment.centerRight,
        color: Colors.red,
        padding: const EdgeInsets.only(right: 16),
        child: const Icon(
          Icons.delete,
          color: Colors.white,
        ),
      ),
      onDismissed: (direction) {
        widget.controller.places.removeAt(index);
        setState(() {});
      },
      child: ListTile(
        title: DropdownButton<String>(
          isExpanded: true,
          value: selectedNode,
          padding: const EdgeInsets.symmetric(horizontal: 16),
          hint: const Text('Select Node'),
          onChanged: (String? newValue) {
            if (newValue != null) {
              widget.controller.places[index] = newValue;
              selectedNode = newValue;
              setState(() {});
            }
          },
          items: Provider.of<RMFProvider>(context, listen: false).getVertices().map<DropdownMenuItem<String>>((Vertice value) {
            return DropdownMenuItem<String>(
              value: value.name,
              child: Text(value.name),
            );
          }).toList(),
        ),
      ),
    );
  }
}
