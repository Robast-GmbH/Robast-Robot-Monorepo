import { useState } from 'react'
import PropTypes from 'prop-types'




const AddTask = ({ onAdd, coords }) => {
        const [text, setText] = useState('')
        const [reminder, setReminder] = useState(false)

        const onSubmit = (e) => {
                e.preventDefault()
                if (!text) {
                        alert('Add stupid text')
                        return
                }

                onAdd({ text, reminder, coords })
                setText('')
                setReminder(false)
        }

        return (
                <form className='add-form' onSubmit={onSubmit}>
                        <div className='form-control'>
                                <label>Order</label>
                                <input type='text' placeholder='What do you want?'
                                        value={text} onChange={(e) => setText(e.target.value)} />
                        </div>

                        <div className='form-control form-control-checkbox'>
                                <label>Deliver by Robot</label>
                                <input
                                        type='checkbox'
                                        checked={reminder}
                                        value={reminder}
                                        onChange={(e) => setReminder(e.currentTarget.checked)} />
                        </div>

                        <div className='form-control form-control-checkbox'>
                                <label>Goal-Coordinates: (debug)</label>
                                {coords.length > 0 ? (
                                        <h1>[{coords[0]}, {coords[1]}]</h1>
                                ) : (
                                        <h1>no coords</h1>
                                )}


                        </div>

                        <input type='submit' value='Save' className='btn btn-block' />
                </form>
        )
}

AddTask.defaultProps = {
        coords: []
}
AddTask.propTypes = {
        coords: PropTypes.array,
        onAdd: PropTypes.func
}
export default AddTask

