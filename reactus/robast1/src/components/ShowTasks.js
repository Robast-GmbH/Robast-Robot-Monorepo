import PropTypes from 'prop-types'
import Tasks from "./Tasks"


const ShowTasks = ({ tasks, onDelete, onToggle }) => {
        return (
                <div>
                        {tasks.length > 0 ? (
                                <Tasks
                                        tasks={tasks}
                                        onDelete={onDelete}
                                        onToggle={onToggle}>
                                </Tasks>
                        ) : (
                                'empty'
                        )}
                </div>
        )
}


ShowTasks.protoTypes = {
        tasks: PropTypes.object,
        onDelete: PropTypes.func,
        onToggle: PropTypes.func,
}

export default ShowTasks
