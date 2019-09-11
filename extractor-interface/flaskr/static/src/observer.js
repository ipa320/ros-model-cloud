import {eventTypes} from "./constants";

class Observer {

    constructor() {
        this.subscriptionsObserver = {};

        for (let type in eventTypes) {
            this.subscriptionsObserver[type] = []
        }
    }

    subscribe = (event, callback) => {
        this.subscriptionsObserver[event].push(callback);
    };

    execute = (event, data) => {
        this.subscriptionsObserver[event].map((callback) => {
            return callback(data)
        })
    };

    unsubscribe = (event, callback) => {
        this.subscriptionsObserver[event] = this.subscriptionsObserver[event].filter((cb) => cb !== callback)
    };
}

export default new Observer()